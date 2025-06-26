import can
import time

bus = can.interface.Bus(channel='can0', interface='socketcan')
responding_nodes = set()

for node_id in range(11):
    msg = can.Message(arbitration_id=0x00 | node_id,  # optional: use proper ODrive ping
                      data=[0x00] * 8,
                      is_extended_id=False)
    try:
        bus.send(msg)
        print(f"Sent ping to node {node_id}")
    except can.CanError:
        print(f"Failed to send to node {node_id}")

print("Listening for responses...")
start_time = time.time()
timeout = 2.0

while time.time() - start_time < timeout:
    msg = bus.recv(timeout=0.1)
    if msg:
        # Properly decode ODrive arbitration ID format
        node_id = (msg.arbitration_id >> 5) & 0x7F
        responding_nodes.add(node_id)

print("Nodes responding:", responding_nodes)
bus.shutdown()
