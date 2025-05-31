model urban_roads_pathfinding
global {
   list<float> vehicle_travel_times <- [];  // Lista para almacenar los tiempos de viaje de los vehículos

    reflex calculate_average_time {
        // Calcular el tiempo medio de los vehículos al final de la simulación
        if (length(vehicle_travel_times) > 0) {
            float total_time <- sum(vehicle_travel_times);
            float average_time <- total_time / length(vehicle_travel_times);
            write "Average travel time of all vehicles: " + average_time + " seconds";
        }
    }
   
    int grid_size <- 20;
    int nb_vehicles <- 20;
    int nb_special_vehicles <- 5;
    int light_cycle <- 20; // 10 seconds per phase
    float grid_offset <- 2.0;
    float collision_distance <- 3.5;
    graph road_network;
    list<float> arrival_times <- [];
    

    init {
        loop i from: 0 to: grid_size - 1 {
            loop j from: 0 to: grid_size - 1 {
                point pos <- {i * 3.0 - (grid_size * 1.5) + grid_offset, j * 3.0 - (grid_size * 1.5) + grid_offset};

                create road_node {
                    grid_x <- i;
                    grid_y <- j;
                    location <- pos;
                }

                if (i mod 3 = 0 and j mod 3 = 0) {
                    create traffic_light {
                        node_x <- i;
                        node_y <- j;
                        location <- pos;
                        timer <- rnd(0, light_cycle / 2);
                        light_state <- flip(0.5) ? "green" : "red";
                    }

                    create sensor {
                        node_x <- i;
                        node_y <- j;
                        location <- pos;
                    }
                }
            }
        }

        road_network <- graph([]);
        loop node over: road_node {
            road_node right_neighbor <- road_node first_with (each.grid_x = node.grid_x + 1 and each.grid_y = node.grid_y);
            if right_neighbor != nil {
                add edge(node::right_neighbor, 1.0) to: road_network;
            }
            road_node bottom_neighbor <- road_node first_with (each.grid_x = node.grid_x and each.grid_y = node.grid_y + 1);
            if bottom_neighbor != nil {
                add edge(node::bottom_neighbor, 1.0) to: road_network;
            }
        }

        list<road_node> available_nodes <- copy(road_node);
        create vehicle number: nb_vehicles {
        	start_cycle <- cycle;
            current_node <- one_of(available_nodes);
            remove current_node from: available_nodes;
            destination <- one_of(road_node where (each != current_node));
            location <- current_node.location;
            path valid_path <- road_network path_between(current_node, destination);
            if (valid_path != nil) {
                route_nodes <- valid_path.vertices;
            } else {
                route_nodes <- [];
            }
        }

        create special_vehicle number: nb_special_vehicles {
        	start_cycle <- cycle;
            current_node <- one_of(available_nodes);
            remove current_node from: available_nodes;
            destination <- one_of(road_node where (each != current_node));
            location <- current_node.location;
            path valid_path <- road_network path_between(current_node, destination);
            if (valid_path != nil) {
                route_nodes <- valid_path.vertices;
            } else {
                route_nodes <- [];
            }
        }
    }

    reflex update_network_weights {
        road_network <- graph([]);
        loop node over: road_node {
            road_node right_neighbor <- road_node first_with (each.grid_x = node.grid_x + 1 and each.grid_y = node.grid_y);
            if right_neighbor != nil {
                float weight <- 1.0;
                sensor nearby_sensor <- one_of(sensor at_distance 3.0);
                if (nearby_sensor != nil and (nearby_sensor.congestion_alert or nearby_sensor.traffic_jam)) {
                    weight <- 10.0;
                }
                add edge(node::right_neighbor, weight) to: road_network;
            }
            road_node bottom_neighbor <- road_node first_with (each.grid_x = node.grid_x and each.grid_y = node.grid_y + 1);
            if bottom_neighbor != nil {
                float weight <- 1.0;
                sensor nearby_sensor <- one_of(sensor at_distance 3.0);
                if (nearby_sensor != nil and (nearby_sensor.congestion_alert or nearby_sensor.traffic_jam)) {
                    weight <- 10.0;
                }
                add edge(node::bottom_neighbor, weight) to: road_network;
            }
        }
    }
}

species road_node {
    int grid_x;
    int grid_y;

    aspect default {
        draw circle(0.3) color: #gray;
    }
}

species traffic_light {
    string light_state <- "green";
    int timer <- 0;
    int node_x;
    int node_y;
    int local_light_cycle <- light_cycle;

    reflex toggle_light {
        timer <- timer + 1;
        if (timer >= 2 * local_light_cycle) {
            timer <- 0;
            light_state <- "green";
        } else if (timer >= local_light_cycle) {
            light_state <- "red";
        }
    }

    reflex adapt_phase when: cycle > 20 {
        sensor nearby_sensor <- one_of(sensor where (each.node_x = node_x and each.node_y = node_y));
        if (nearby_sensor != nil and (nearby_sensor.traffic_density > 5 or nearby_sensor.traffic_jam)) {
            local_light_cycle <- 20;
        } else {
            local_light_cycle <- light_cycle;
        }
    }

    aspect default {
        draw square(0.5) color: (light_state = "green" ? #green : #red);
    }
}

species sensor {
    int node_x;
    int node_y;
    float traffic_density <- 0.0;
    float avg_speed <- 0.0;
    bool congestion_alert <- false;
    bool traffic_jam <- false;
    int jam_cooldown <- 0;

    reflex monitor_traffic when: jam_cooldown = 0 {
        list<vehicle> nearby_vehicles <- (vehicle at_distance 3.0) + (special_vehicle at_distance 3.0);
        traffic_density <- length(nearby_vehicles);
        if (length(nearby_vehicles) > 0) {
            avg_speed <- mean(nearby_vehicles collect each.speed);
        } else {
            avg_speed <- 0.0;
        }
        congestion_alert <- traffic_density > 5;
        traffic_jam <- avg_speed < 0.01 and traffic_density > 3;

        if (congestion_alert) {
            write "Congestion at (" + node_x + "," + node_y + "), density: " + traffic_density;
        }
        if (traffic_jam) {
            write "Traffic jam at (" + node_x + "," + node_y + "), avg speed: " + avg_speed;
            jam_cooldown <- 10;
        }

        if (congestion_alert or traffic_jam) {
            ask nearby_vehicles {
                path valid_path <- road_network path_between(current_node, destination);
                if (valid_path != nil) {
                    route_nodes <- valid_path.vertices;
                    route_index <- 1;
                }
            }
        }
    }

    reflex update_cooldown when: jam_cooldown > 0 {
        jam_cooldown <- jam_cooldown - 1;
    }

    reflex broadcast_data when: congestion_alert or traffic_jam {
        ask traffic_light where (each.node_x = node_x and each.node_y = node_y) {
            // Data used in adapt_phase reflex
        }
        ask (vehicle at_distance 3.0) + (special_vehicle at_distance 3.0) {
            if (myself.traffic_jam) {
                speed <- speed * 0.5;
            }
        }
    }

    aspect default {
        draw polygon([{-0.4,0}, {0,0.4}, {0.4,0}, {0,-0.4}]) color: (congestion_alert or traffic_jam ? #red : #purple);
    }
}

species vehicle {
    road_node current_node;
    road_node destination;
    list<road_node> route_nodes;
    int route_index <- 1;
    int start_cycle;
    float speed <- 0.6;
    int wait_cycles <- 0;
    int collision_count <- 0;
    point last_collision_loc <- nil;
    float travel_time <- 0.0;

    reflex follow_route when: route_nodes != nil and wait_cycles = 0 {
        
        if (route_index < length(route_nodes)) {
            road_node next_node <- route_nodes[route_index];

            traffic_light light <- one_of(traffic_light where (each.node_x = next_node.grid_x and each.node_y = next_node.grid_y));
            if (light != nil and light.light_state = "red") {
                write "Vehicle " + name + " stopped at red light at (" + next_node.grid_x + "," + next_node.grid_y + ")";
                return;
            }

            point next_loc <- next_node.location;
            list<vehicle> occupying_vehicles <- (vehicle at_distance 3.0) where (each != self and each.route_nodes != nil and length(each.route_nodes) > each.route_index and each.route_nodes[each.route_index] = next_node and each.location distance_to next_loc < collision_distance);
            list<special_vehicle> occupying_specials <- (special_vehicle at_distance 3.0) where (each != self and each.route_nodes != nil and length(each.route_nodes) > each.route_index and each.route_nodes[each.route_index] = next_node and each.location distance_to next_loc < collision_distance);
            bool occupied <- !empty(occupying_vehicles) or !empty(occupying_specials);
            if (occupied) {
                // Check if the occupying vehicle is moving
                bool occupying_is_moving <- empty(occupying_vehicles + occupying_specials where (each.wait_cycles > 0));
                // Decide which vehicle has priority based on name (lower name wins)
                agent occupying_agent <- empty(occupying_vehicles + occupying_specials) ? nil : (occupying_vehicles + occupying_specials with_min_of (each.name))[0];
                bool i_have_priority <- occupying_agent = nil or name < occupying_agent.name;

                write "Vehicle " + name + " detected collision at " + next_loc + ", occupying agent: " + (occupying_agent != nil ? occupying_agent.name : "none") + ", occupying moving: " + occupying_is_moving + ", I have priority: " + i_have_priority;

                if (occupying_is_moving or i_have_priority) {
                    // If the occupying vehicle is moving or I have priority, I wait
                    wait_cycles <- rnd(3, 6);
                    collision_count <- collision_count + 1;
                    if (last_collision_loc != nil and last_collision_loc = next_loc) {
                        collision_count <- collision_count + 1;
                    }
                    last_collision_loc <- next_loc;
                    if (collision_count > 2) { // Reduced threshold for rerouting
                        write "Vehicle " + name + " rerouting due to repeated collisions";
                        collision_count <- 0;
                        last_collision_loc <- nil;
                        list<road_node> candidates <- road_node where (each != current_node and each.location distance_to location > 6.0);
                        if (!empty(candidates)) {
                            destination <- one_of(candidates);
                            path valid_path <- road_network path_between(current_node, destination);
                            if (valid_path != nil) {
                                route_nodes <- valid_path.vertices;
                                route_index <- 1;
                                wait_cycles <- 0; // Allow immediate movement after rerouting
                            } else {
                                wait_cycles <- rnd(10, 15); // Wait longer if no path found
                            }
                        }
                    }
                } else {
                    // If the occupying vehicle is stopped and I don't have priority, I reroute immediately
                    write "Vehicle " + name + " rerouting because occupying vehicle is stopped";
                    list<road_node> candidates <- road_node where (each != current_node and each.location distance_to location > 6.0);
                    if (!empty(candidates)) {
                        destination <- one_of(candidates);
                        path valid_path <- road_network path_between(current_node, destination);
                        if (valid_path != nil) {
                            route_nodes <- valid_path.vertices;
                            route_index <- 1;
                        } else {
                            wait_cycles <- rnd(3, 6); // Wait if no path found
                        }
                    }
                }
                return;
            }

            float dist <- location distance_to next_loc;
            if (dist > 0.2) {
                point direction <- (next_loc - location) / dist;
                location <- location + direction * min(speed, dist);
                travel_time <- travel_time + (min(speed, dist) / speed);  // Incrementar el tiempo de viaje
            }

            if (dist <= 0.2) {
                current_node <- next_node;
                route_index <- route_index + 1;
                collision_count <- 0;
                last_collision_loc <- nil;
            }
        } else {
            // Llegó al destino - almacenar el tiempo de viaje en la lista global
            write "Vehicle " + name + " reached destination at " + location + ". Travel time: " + travel_time + " seconds";
            vehicle_travel_times <- vehicle_travel_times + [travel_time];  // Almacenar el tiempo de viaje
            do die;
        }
    }

    reflex reroute when: wait_cycles > 0 {
        wait_cycles <- wait_cycles - 1;

        if (wait_cycles = 0) {
            current_node <- destination;
            list<road_node> candidates <- road_node where (each != current_node and each.location distance_to location > 6.0);
            float min_density <- 100.0;
            road_node best_candidate <- nil;
            loop cand over: candidates {
                int nearby_vehicles <- length(list(vehicle) + list(special_vehicle) where (each.location distance_to cand.location < 6.0));
                if (nearby_vehicles < min_density) {
                    min_density <- nearby_vehicles;
                    best_candidate <- cand;
                }
            }
            if (best_candidate != nil) {
                destination <- best_candidate;
            } else {
                list<road_node> weighted_candidates <- candidates;
                loop cand over: weighted_candidates {
                    int vehicles_ahead <- length(list(vehicle) + list(special_vehicle) where (each.location distance_to cand.location < 6.0 and each.route_nodes contains cand));
                    if (vehicles_ahead > 0) {
                        weighted_candidates <- weighted_candidates - cand;
                    }
                }
                destination <- one_of(weighted_candidates);
            }
            path valid_path <- road_network path_between(current_node, destination);
            if (valid_path != nil) {
                route_nodes <- valid_path.vertices;
                route_index <- 1;
            } else {
                route_nodes <- [];
                route_index <- 1;
            }
        }
    }

    aspect default {
        draw circle(0.5) color: #blue;
    }
}
species special_vehicle parent: vehicle {
    bool emergency_mode <- false;

    reflex activate_emergency when: flip(0.001) {
        emergency_mode <- !emergency_mode;
        if (emergency_mode) {
            write "Special vehicle " + name + " entered emergency mode";
        }
    }

    reflex follow_route when: route_nodes != nil and wait_cycles = 0 {
        if (route_index < length(route_nodes)) {
            road_node next_node <- route_nodes[route_index];

            if (emergency_mode) {
                ask traffic_light where (each.node_x = next_node.grid_x and each.node_y = next_node.grid_y) {
                    light_state <- "green";
                    timer <- 0;
                }
                ask vehicle at_distance 2.0 {
                    wait_cycles <- 3;
                    write "Vehicle " + name + " paused by special vehicle at " + location;
                }
                ask special_vehicle at_distance 2.0 where (each != self and each.name > name) {
                    wait_cycles <- 3;
                    write "Special vehicle " + name + " paused by higher-priority special vehicle";
                }
            } else {
                traffic_light light <- one_of(traffic_light where (each.node_x = next_node.grid_x and each.node_y = next_node.grid_y));
                if (light != nil and light.light_state = "red") {
                    write "Special vehicle " + name + " stopped at red light at (" + next_node.grid_x + "," + next_node.grid_y + ")";
                    return;
                }
            }

            point next_loc <- next_node.location;
            list<vehicle> occupying_vehicles <- (vehicle at_distance 3.0) where (each != self and each.route_nodes != nil and length(each.route_nodes) > each.route_index and each.route_nodes[each.route_index] = next_node and each.location distance_to next_loc < collision_distance);
            list<special_vehicle> occupying_specials <- (special_vehicle at_distance 3.0) where (each != self and each.route_nodes != nil and length(each.route_nodes) > each.route_index and each.route_nodes[each.route_index] = next_node and each.location distance_to next_loc < collision_distance);
            bool occupied <- !empty(occupying_vehicles) or !empty(occupying_specials);
            if (occupied and !emergency_mode) {
                // Check if the occupying vehicle is moving
                bool occupying_is_moving <- empty(occupying_vehicles + occupying_specials where (each.wait_cycles > 0));
                // Decide which vehicle has priority based on name (lower name wins)
                agent occupying_agent <- empty(occupying_vehicles + occupying_specials) ? nil : (occupying_vehicles + occupying_specials with_min_of (each.name))[0];
                bool i_have_priority <- occupying_agent = nil or name < occupying_agent.name;

                write "Special vehicle " + name + " detected collision at " + next_loc + ", occupying agent: " + (occupying_agent != nil ? occupying_agent.name : "none") + ", occupying moving: " + occupying_is_moving + ", I have priority: " + i_have_priority;

                if (occupying_is_moving or i_have_priority) {
                    // If the occupying vehicle is moving or I have priority, I wait
                    wait_cycles <- rnd(3, 6);
                    collision_count <- collision_count + 1;
                    if (last_collision_loc != nil and last_collision_loc = next_loc) {
                        collision_count <- collision_count + 1;
                    }
                    last_collision_loc <- next_loc;
                    if (collision_count > 2) { // Reduced threshold for rerouting
                        write "Special vehicle " + name + " rerouting due to repeated collisions";
                        collision_count <- 0;
                        last_collision_loc <- nil;
                        list<road_node> candidates <- road_node where (each != current_node and each.location distance_to location > 6.0);
                        if (!empty(candidates)) {
                            destination <- one_of(candidates);
                            path valid_path <- road_network path_between(current_node, destination);
                            if (valid_path != nil) {
                                route_nodes <- valid_path.vertices;
                                route_index <- 1;
                                wait_cycles <- 0; // Allow immediate movement after rerouting
                            } else {
                                wait_cycles <- rnd(10, 15); // Wait longer if no path found
                            }
                        }
                    }
                } else {
                    // If the occupying vehicle is stopped and I don't have priority, I reroute immediately
                    write "Special vehicle " + name + " rerouting because occupying vehicle is stopped";
                    list<road_node> candidates <- road_node where (each != current_node and each.location distance_to location > 6.0);
                    if (!empty(candidates)) {
                        destination <- one_of(candidates);
                        path valid_path <- road_network path_between(current_node, destination);
                        if (valid_path != nil) {
                            route_nodes <- valid_path.vertices;
                            route_index <- 1;
                        } else {
                            wait_cycles <- rnd(3, 6); // Wait if no path found
                        }
                    }
                }
                return;
            }

            float dist <- location distance_to next_loc;
            if (dist > 0.2) {
                point direction <- (next_loc - location) / dist;
                location <- location + direction * min(speed, dist);
            }

            if (dist <= 0.2) {
                current_node <- next_node;
                route_index <- route_index + 1;
                collision_count <- 0;
                last_collision_loc <- nil;
            }
        } else {
            // Llegó al destino - desaparecer
            write "Special vehicle " + name + " reached destination at " + location;
            do die;
        }
    }

    aspect default {
        draw triangle(0.7) color: (emergency_mode ? #yellow : #green);
    }
}

experiment follow_roads type: gui {
    output {
        display map type: opengl refresh: true {
            camera 'default' location: {(grid_size * 1.5), (grid_size * 1.5), 60} dynamic: false;
            species road_node;
            species traffic_light;
            species sensor;
            species vehicle;
            species special_vehicle;

            agents "roads" value: road_network.edges {
                draw shape color: #black;
            }
        }
        monitor "Congestion Alerts" value: string(length(sensor where (each.congestion_alert))) + " sensors report congestion";
        monitor "Traffic Jams" value: string(length(sensor where (each.traffic_jam))) + " sensors report jams";
    }
}