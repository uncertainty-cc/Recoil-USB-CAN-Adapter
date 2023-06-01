import argparse

def getID():
    parser = argparse.ArgumentParser()
    parser.add_argument("id", help="CAN ID value", default="1")
    args = parser.parse_args()
    device_id = int(args.id.split("=")[-1])
    return device_id
