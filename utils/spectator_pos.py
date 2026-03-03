import carla
import time 
_HOST_ = '127.0.0.1'
_PORT_ = 2000
_SLEEP_TIME_ = 1


def main():
	client = carla.Client(_HOST_, _PORT_)
	client.set_timeout(2.0)
	world = client.get_world()
	
	actors = world.get_actors()
	for actor in actors:
		if actor.type_id == 'traffic.traffic_light':
			lb = actor.get_light_boxes()
			lightbox_info = [
                {
                    'bottom_position': (i.location.x, i.location.y, i.location.z - i.extent.z),
                    'z_height': i.extent.z ,
                    'y_1': i.location.y - 0.15 ,
                    'y_2': i.location.y +0.15
                }
                for i in lb
            ]
			#print('Actor Bulbs: {}'.format(lightbox_info))
			print('Actor Position: ', actor.get_transform().location)
			print('Actor Orientation: ', actor.get_transform().rotation)
			
			
	

	
    #Code to publish current pos
	#while(True):
	#	t = world.get_spectator().get_transform()
	#	# coordinate_str = "(x,y) = ({},{})".format(t.location.x, t.location.y)
	#	coordinate_str = "(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z)
	#	print (coordinate_str)
	#	time.sleep(_SLEEP_TIME_)




if __name__ == '__main__':
	main()