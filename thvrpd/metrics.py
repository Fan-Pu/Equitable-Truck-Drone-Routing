from __future__ import annotations

from typing import Any

from .routes import PAYLOAD_TOLERANCE


def solution_service_metrics(instance, result) -> dict[str, Any]:
    result_record = result.to_record()
    counts = {customer: 0 for customer in instance.customers}
    for route in result.routes:
        for customer in route.served:
            counts[customer] += 1
    delays = {
        customer: result_record["customer_service_times"][customer] - result.objective.bounds.arrival_lb[customer]
        for customer in instance.customers
        if customer in result_record["customer_service_times"]
    }
    wait_values = []
    for route in result.routes:
        for hub, customers in route.drone_blocks.items():
            if customers:
                wait_values.append(max(instance.drone_trip_time[(hub, customer)] for customer in customers))
    payloads = [sum(instance.demand[customer] for customer in route.served) for route in result.routes]
    total_customer_demand = sum(instance.demand[customer] for customer in instance.customers)
    available_truck_payload_total = instance.num_trucks * instance.truck_payload
    return {
        "service_feasible": all(value == 1 for value in counts.values()) and len(result.routes) <= instance.num_trucks,
        "coverage_counts": counts,
        "selected_trucks": len(result.routes),
        "available_trucks": instance.num_trucks,
        "drone_sorties": sum(route.drone_sorties for route in result.routes),
        "available_drones_total": instance.num_trucks * instance.drones_per_truck,
        "truck_served_customers": sum(len(route.truck_served) for route in result.routes),
        "drone_served_customers": sum(len(route.drone_served) for route in result.routes),
        "payloads": payloads,
        "max_payload": max(payloads, default=0.0),
        "payload_feasible": all(payload <= instance.truck_payload + PAYLOAD_TOLERANCE for payload in payloads),
        "total_customer_demand": total_customer_demand,
        "available_truck_payload_total": available_truck_payload_total,
        "mean_delay": sum(delays.values()) / len(delays) if delays else None,
        "max_delay": max(delays.values()) if delays else None,
        "delay_square_sum": sum(value * value for value in delays.values()),
        "waiting_blocks": len(wait_values),
        "total_wait_time": sum(wait_values),
        "mean_wait_time": sum(wait_values) / len(wait_values) if wait_values else 0.0,
        "max_wait_time": max(wait_values, default=0.0),
    }
