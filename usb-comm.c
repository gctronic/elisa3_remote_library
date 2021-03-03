#include "usb-comm.h"
#if defined(_WIN32) || defined(_WIN64)
    #include "libusb.h"
#endif
#ifdef __APPLE__
    #include </opt/local/include/libusb-1.0/libusb.h>
#endif
#if defined(__linux__)
    #include <libusb-1.0/libusb.h>
#endif

static struct libusb_device_handle *devh = NULL;

void get_device_list(void) {
    libusb_device **devs;
    ssize_t count = libusb_get_device_list(NULL, &devs);
    size_t idx = 0;
    struct libusb_device_descriptor desc = {0};

    for (idx=0; idx<count; ++idx) {
        libusb_get_device_descriptor(devs[idx], &desc);
        printf("Vendor:Device = %04x:%04x\n", desc.idVendor, desc.idProduct);
   }
}

static int find_nrf_device(void) {
	devh = (libusb_device_handle*)libusb_open_device_with_vid_pid(NULL, 0x1915, 0x0101);
	return devh ? 0 : -1;
}

int usb_send(char* data, int nbytes) {

	int transferred = 0;
	int r = 0;

	r = libusb_bulk_transfer(devh, 0x01, (unsigned char*)data, 64, &transferred, 5000); // address 0x01
	if (r < 0) {
		fprintf(stderr, "bulk write error %d\n", r);
		return r;
	}
	if (transferred < nbytes) {
		fprintf(stderr, "short write (%d)\n", r);
		return -1;
	}

	return 0;

}

int usb_receive(char* data, int nbytes) {

	int received = 0;
	int r = 0;

	r = libusb_bulk_transfer(devh, 0x81, (unsigned char*)data, nbytes, &received, 5000);
	if (r < 0) {
		fprintf(stderr, "bulk read error %d\n", r);
		return r;
	}
	if (received < nbytes) {
		fprintf(stderr, "short read (%d)\n", r);
		return -1;
	}

    return 0;

}

int openCommunication() {
    int error=0;

	error = libusb_init(NULL);
	if (error < 0) {
	    fprintf(stderr, "libusb_init error %d\n", error);
		return error;
	}

	//libusb_set_debug(NULL, 3);

	//get_device_list();

	error = find_nrf_device();
	if (error < 0) {
		fprintf(stderr, "Could not find/open device\n");
	}

	error = libusb_claim_interface(devh, 0);
	if (error < 0) {
		fprintf(stderr, "usb_claim_interface error %d\n", error);
	}

	return 0;
}

void closeCommunication() {
	libusb_release_interface(devh, 0);
	libusb_close(devh);
	libusb_exit(NULL);
}

