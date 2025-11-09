import asyncio
import logging
from ble_serial.bluetooth.ble_client import BLE_client

# === Your HM-10 parameters ===
ADAPTER = "hci0"                           # Linux; ignore on Windows
DEVICE = "D4:36:39:61:CF:5B"               # MAC of your module
SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB"
WRITE_UUID   = "0000FFE1-0000-1000-8000-00805F9B34FB"
READ_UUID    = "0000FFE1-0000-1000-8000-00805F9B34FB"
WRITE_WITH_RESPONSE = True


# === Callback for incoming data ===
def receive_callback(value: bytes):
    """Called when data is received from the HM-10."""
    try:
        print("Received:", value.decode(errors="ignore").strip())
    except Exception:
        print("Received (raw):", value)


# === Periodic sender ===
async def number_sender(ble: BLE_client):
    """Send an increasing number every 5 seconds."""
    counter = 1
    while True:
        message = f"{counter}\n".encode("utf-8")
        ble.queue_send(message)
        print(f"Sent: {counter}")
        print("------------")
        await asyncio.sleep(5.0)


# === Main BLE logic ===
async def run_ble():
    """Main connection loop with auto-reconnect."""
    ble = BLE_client(ADAPTER, "HM10")
    ble.set_receiver(receive_callback)

    while True:
        try:
            print(f"Connecting to {DEVICE}...")
            await ble.connect(DEVICE, "public", SERVICE_UUID, 10.0)
            await ble.setup_chars(WRITE_UUID, READ_UUID, "rw", WRITE_WITH_RESPONSE)
            print("Connected to HM-10!")

            # Run send & BLE background tasks concurrently
            await asyncio.gather(
                ble.send_loop(),
                number_sender(ble)
            )

        except Exception as e:
            print(f"⚠️ Connection error: {e}")
            print("Reconnecting in 1 second...")
            await asyncio.sleep(1)

        finally:
            await ble.disconnect()
            print("Disconnected. Retrying...")

            await asyncio.sleep(2)


# === Run script ===
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    try:
        asyncio.run(run_ble())
    except KeyboardInterrupt:
        print("Program stopped.")
