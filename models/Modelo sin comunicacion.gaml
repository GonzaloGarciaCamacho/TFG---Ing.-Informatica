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
    int nb_vehicles <- 10;
    int nb_special_vehicles <- 1;
    int light_cycle <- 10; // 10 segundos por fase
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
                add edge(node::right_neighbor, 1.0) to: road_network;
            }
            road_node bottom_neighbor <- road_node first_with (each.grid_x = node.grid_x and each.grid_y = node.grid_y + 1);
            if bottom_neighbor != nil {
                add edge(node::bottom_neighbor, 1.0) to: road_network;
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
            light_state <- (light_state = "green" ? "red" : "green"); // Alterna entre rojo y verde
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
        // No más comunicaciones entre sensores y otros agentes
        traffic_density <- 0.0;
        avg_speed <- 0.0;
        congestion_alert <- false;
        traffic_jam <- false;
    }

    aspect default {
        draw polygon([{-0.4,0}, {0,0.4}, {0.4,0}, {0,-0.4}]) color: #purple;
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
            bool occupied <- !empty(occupying_vehicles);

            // Si el siguiente nodo está ocupado, se ralentiza la velocidad
            if (occupied) {
                // Reducir la velocidad dependiendo de cuántos vehículos ocupen el siguiente nodo
                int density <- length(occupying_vehicles);
                speed <- 0.2 * density;  // Ajustar la velocidad según la densidad del tráfico
                write "Vehicle " + name + " slowing down due to traffic at (" + next_node.grid_x + "," + next_node.grid_y + ")";
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
    }

    aspect default {
        draw circle(0.5) color: #blue;
    }
}

species special_vehicle parent: vehicle {
    bool emergency_mode <- false;

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
