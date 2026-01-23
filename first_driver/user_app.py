def main():
    driver_handle = open('/proc/ty_driver')
    msg_from_ks = driver_handle.readline()
    print(msg_from_ks)
    return

main()
