/*
 * Copyright 2008, The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * Heavily modified by HUSSON Pierre-Hugues <phhusson@free.fr> for raphael/diamond
 */

#define LOG_TAG "Sensors"

#include <hardware/hardware.h>
#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <linux/input.h>
#include <cutils/log.h>
#include <cutils/atomic.h>

/*****************************************************************************/


#define EVENT_TYPE_ACCEL_X          ABS_X
#define EVENT_TYPE_ACCEL_Y          ABS_Z
#define EVENT_TYPE_ACCEL_Z          ABS_Y
#define EVENT_TYPE_ACCEL_STATUS     ABS_WHEEL

#define EVENT_TYPE_TEMPERATURE      ABS_THROTTLE
#define EVENT_TYPE_STEP_COUNT       ABS_GAS

#define SENSOR_STATE_MASK           (0x7FFF)

//#define DEBUG /* Uncommit this if You need to debug sensors */

char phys[20];
char axis_order[3];

void parse_axis_order() {
	int fd=open("/proc/cmdline", O_RDONLY);
	char buf[1024];
	char *ptr,*ptr2;
	int i;
	buf[1023]=0;
	if(fd<0)
		return;
	if(read(fd, buf, 1023)<0) {
		close(fd);
		return;
	}
	ptr=strstr(buf, "gsensor_axis=");
	if(!ptr)
		return;
	if(ptr!=buf)
		if(ptr[-1]!=' ')
			return;
	ptr+=strlen("gesnsor_axis=");

	for(i=0;i<3;++i) {
		ptr2=index(ptr, ',');
		if(!ptr2)
			ptr2=index(ptr, ' ');
		if(ptr2)
			*ptr2=0;
		axis_order[i]=atoi(ptr);
		ptr=ptr2+1;
	}
}

int open_input() {
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;
    dir = opendir(dirname);
    if(dir == NULL)
        return -1;
    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while((de = readdir(dir))) {
        if(de->d_name[0] == '.' &&
           (de->d_name[1] == '\0' ||
            (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        strcpy(filename, de->d_name);
        fd = open(devname, O_RDONLY);
        if (fd>=0) {
            char name[80];
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
                name[0] = '\0';
            }
            if (!strcmp(name, "proximity_gp2ap")) {
		    bzero(phys, 20);
		ioctl(fd, EVIOCGPHYS(sizeof(phys)-1), &phys);
		if(strcmp(phys, "i2c/0-0019")==0) {
			axis_order[0]=-1;
			axis_order[1]=-2;
			axis_order[2]=3;
		} else {
			axis_order[0]=1;
			axis_order[1]=-2;
			axis_order[2]=3;
		}
		parse_axis_order();


                LOGD("using %s (name=%s,phys=%s)", devname, name, phys);
                break;
            }
            close(fd);
            fd = -1;
        }
    }
    closedir(dir);
    return fd;
}

int sFD=-1;
int evdev_close(struct hw_device_t *dev) {
#ifdef DEBUG
	LOGE("called evdev_close");
#endif
	close(sFD);
	sFD=-1;
	return 0;
}

native_handle_t *open_data_source(struct sensors_control_device_t *dev) {
	native_handle_t *hdl=native_handle_create(1,0);
	hdl->data[0]=sFD;
#ifdef DEBUG
	LOGE("called open_data_source");
#endif
	return hdl;
}

int activated=0;
int activate(struct sensors_control_device_t *dev, int handle, int enabled) {
#ifdef DEBUG
	LOGE("sensors activate handle=%d, enabled=%d", handle, enabled);
#endif
	if(enabled)
		activated|=1<<handle;
	else
		activated&=~(1<<handle);

	enabled=!!( ( activated&(1<<SENSOR_TYPE_ACCELEROMETER)) | 
			(activated&(1<<SENSOR_TYPE_ORIENTATION))); 

	char *filename;
	asprintf(&filename, "/sys/class/i2c-adapter/i2c-5/%s/enable", phys+4);
	FILE *fd=fopen(filename, "w");
	free(filename);
	if(!fd)
		return -1;
	fprintf(fd, "%d\n", enabled);
	fclose(fd);

	LOGE("sensors activate returing 1");
	return 1;
}

int set_delay(struct sensors_control_device_t *dev, int32_t ms) {
#ifdef DEBUG
	LOGE("sensors set_delay(%d)", ms);
#endif
	if(ms==0) {
		ms=100;
	}
	char *filename;
	asprintf(&filename, "/sys/class/i2c-adapter/i2c-5/%s/rate", phys+4);
	FILE *fd=fopen(filename, "w");
	if(!fd)
		return -1;
	fprintf(fd, "%d\n", 10000/ms);
	fclose(fd);
	return 1;
}

int wake(struct sensors_control_device_t *dev) {
#ifdef DEBUG
	LOGE("called wake");
#endif
	return 0;
}

int data_open(struct sensors_data_device_t *dev, native_handle_t* nh) {
#ifdef DEBUG
	LOGE("Called data_open");
#endif
	return 0;
}

int data_close(struct sensors_data_device_t *dev, native_handle_t* nh) {
#ifdef DEBUG
	LOGE("Called data close");
#endif
	return 0;
}

double read_light() {
	int fd=open("/dbgfs/e0_dbg/light", O_RDONLY);
	char buf[256];
	int i;
	buf[255]=0;
	if(fd<0)
		return -10;
	if(read(fd, buf, 255)<0) {
		close(fd);
		return -10;
	}

	i=atoi(buf);
	close(fd);
	return i*0.1L;
}

double read_prox() {
	int fd=open("/dbgfs/e0_navi/proximity", O_RDONLY);
	char buf[256];
	int res;
	buf[255]=0;
	if(fd<0)
		return -10;
	if(read(fd, buf, 255)<0) {
		close(fd);
		return -10;
	}
	res=atoi(buf);
	close(fd);
	if(res==0)
		return 10000000.0;
	return 1.0/res;
}

#define MS2 100.0
int oldx=0,oldy=0,oldz=0;
int state=0;
int data_poll(struct sensors_data_device_t *dev, sensors_data_t* data) {
	struct input_event event;
	int ret;
	double value;
#ifdef DEBUG
	LOGE("called data poll");
#endif
	data->time=time(NULL)*1000*1000*1000;

	if(state==1) {
#ifdef DEBUG
		LOGE("Got accel data, send orientation");
#endif
		state=2;
		data->sensor=SENSOR_TYPE_ORIENTATION;
		value=sqrt(oldx*oldx+oldy*oldy+oldz*oldz);
		data->orientation.azimuth=0;
		data->orientation.pitch=asin((float)oldy/(float)value)*180/3.14159;

		if(oldz>0)
			data->orientation.pitch=180-data->orientation.pitch;
		if(data->orientation.pitch>180)
			data->orientation.pitch-=360;
		data->orientation.roll=asin((float)oldx/(float)value)*180/3.14159;
#ifdef DEBUG
		LOGE("Orientation %f,%f,%f\n", data->acceleration.x, data->acceleration.y, data->acceleration.z);
#endif
		return SENSOR_TYPE_ORIENTATION;
	} else if(state==2) {
		LOGE("Sent orientation, send light");
		state=3;
		value=read_light();
		if(value<0) {
			LOGE("Failed orientation & light");
			return data_poll(dev, data);
		}
		data->sensor=SENSOR_TYPE_LIGHT;
		data->light=value;
		return SENSOR_TYPE_LIGHT;
	} else if(state==3) {
		LOGE("Sent light, send proximity");
		state=0;
		value=read_prox();
		if(value<0) {
			LOGE("Failed light & proximity");
			return data_poll(dev, data);
		}
		data->sensor=SENSOR_TYPE_PROXIMITY;
		data->distance=value;
		return SENSOR_TYPE_PROXIMITY;
	}
	if(sFD<0)
		sFD=open_input();
	
	int got=0;
	while(!got) {
		ret=read(sFD, &event, sizeof(event));
		if(ret<0) {
			close(sFD);
			sFD=-1;
			return 0;
		}
		if(ret<sizeof(event))
			continue;
#ifdef DEBUG
		LOGE("Event type: %d/%d/%d", event.type, event.code, event.value);
#endif
		switch(event.type) {
			case EV_SYN:
				got=1;
				state=1;
				break;
			case EV_ABS:
				if(event.code==ABS_Y) 
					oldx=event.value;
				if(event.code==ABS_X)
					oldy=event.value;
				if(event.code==ABS_Z)
					oldz=event.value;
				break;
			default:
				LOGE("Unknown event type: %d/%d/%d", event.type, event.code, event.value);
				break;
		};
	}
	double dat[3];
	dat[0]=oldx/MS2;
	dat[1]=oldy/MS2;
	dat[2]=oldz/MS2;

	data->acceleration.x=(axis_order[0]>0) ? dat[axis_order[0]] : -dat[-axis_order[0]];
	data->acceleration.y=(axis_order[1]>0) ? dat[axis_order[1]] : -dat[-axis_order[1]];
	data->acceleration.z=(axis_order[2]>0) ? dat[axis_order[2]] : -dat[-axis_order[2]];
#ifdef DEBUG
	LOGE("Returing %f,%f,%f\n", data->acceleration.x, data->acceleration.y, data->acceleration.z);
#endif
	return SENSOR_TYPE_ACCELEROMETER;
}

int evdev_open(const struct hw_module_t* module, const char* id, struct hw_device_t** device) {
#ifdef DEBUG
	LOGE("open");
#endif
	if(strcmp(id, SENSORS_HARDWARE_CONTROL)==0) {
#ifdef DEBUG
		LOGE("opening ctl");
#endif
		struct sensors_control_device_t *dev;
		*device=malloc(sizeof(struct sensors_control_device_t));
		memset(*device, 0, sizeof(struct sensors_control_device_t));
		(*device)[0].tag=HARDWARE_DEVICE_TAG;
		(*device)[0].version=0;
		(*device)[0].module=module;
		(*device)[0].close=evdev_close;
		dev=*device;

		dev[0].open_data_source=open_data_source;
		dev[0].activate=activate;
		dev[0].set_delay=set_delay;
		dev[0].wake=wake;
		int i;
		if(sFD<0)
			sFD=open_input();
		if(sFD<0)
			return -1;
#ifdef DEBUG
		LOGE("ctl success opened");
#endif
		return 1;
	} else if(strcmp(id, SENSORS_HARDWARE_DATA)==0) {
#ifdef DEBUG
		LOGE("open hw data");
#endif
		struct sensors_data_device_t *dev;
		*device=malloc(sizeof(struct sensors_data_device_t));
		memset(*device, 0, sizeof(struct sensors_data_device_t));
		(*device)[0].tag=HARDWARE_DEVICE_TAG;
		(*device)[0].version=0;
		(*device)[0].module=module;
		(*device)[0].close=evdev_close;
		dev=*device;

		dev[0].data_open=data_open;
		dev[0].data_close=data_close;
		dev[0].poll=data_poll;
		if(sFD<0)
			sFD=open_input();
#ifdef DEBUG
		LOGE("success");
#endif
		return 1;
	} else {
		return -1;
	}
}

struct sensor_t sensors[]={
	{
		name		: "accelerometer",
		vendor		: "LGE",
		version		: 1,
		handle 		: SENSOR_TYPE_ACCELEROMETER,
		type		: SENSOR_TYPE_ACCELEROMETER,
		maxRange	: 20,
		resolution	: 0.1,
		power		: 42,
	},
	{
		name		: "gyroscope",
		vendor		: "LGE",
		version		: 1,
		handle 		: SENSOR_TYPE_GYROSCOPE,
		type		: SENSOR_TYPE_GYROSCOPE,
		maxRange	: 20,
		resolution	: 0.1,
		power		: 42,
	},
	{
		name		: "batt temp",
		vendor		: "LGE",
		version		: 1,
		handle 		: SENSOR_TYPE_TEMPERATURE,
		type		: SENSOR_TYPE_TEMPERATURE,
		maxRange	: 20,
		resolution	: 0.1,
		power		: 42,
	},
	{
		name		: "orientation",
		vendor		: "LGE",
		version		: 1,
		handle 		: SENSOR_TYPE_ORIENTATION,
		type		: SENSOR_TYPE_ORIENTATION,
		maxRange	: 20,
		resolution	: 0.1,
		power		: 42,
	},
	{
		name		: "light",
		vendor		: "LGE",
		version		: 1,
		handle 		: SENSOR_TYPE_LIGHT,
		type		: SENSOR_TYPE_LIGHT,
		maxRange	: 20,
		resolution	: 0.1,
		power		: 42,
	},
	{
		name		: "pressure",
		vendor		: "LGE",
		version		: 1,
		handle 		: SENSOR_TYPE_PRESSURE,
		type		: SENSOR_TYPE_PRESSURE,
		maxRange	: 20,
		resolution	: 0.1,
		power		: 42,
	},
	{
		name		: "proximity",
		vendor		: "LGE",
		version		: 1,
		handle 		: SENSOR_TYPE_PROXIMITY,
		type		: SENSOR_TYPE_PROXIMITY,
		maxRange	: 20,
		resolution	: 0.1,
		power		: 42,
	},
	     
};

int get_sensors_list(struct sensors_module_t *module, struct sensor_t const** list) {
	LOGD("get_sensors_list");
	*list=sensors;
	return 7;
}

struct hw_module_methods_t evdev_methods = {
	open : evdev_open,
};

const struct sensors_module_t HAL_MODULE_INFO_SYM = {
	common : {
		tag		: HARDWARE_MODULE_TAG,
		version_major	: 1,
		version_minor	: 0,
		id		: SENSORS_HARDWARE_MODULE_ID,
		name		: "accelerometer driver",
		author		: "fergy <ramon.rebersak@gmail.com>",
		methods		: &evdev_methods,
	},
	get_sensors_list : get_sensors_list,
};
