# epuck.inc
# rough model of the epuck robots

define epuckIR sensor
(
  # define the size of each transducer [xsize ysize zsize] in meters
  size [0.002 0.002 0.002] 
  # define the range bounds [min max]
  range [0.0064 1]
  # define the angular field of view in degrees
  fov 10
  # define the number of samples spread over the fov
  samples 1
)

#ranger sensors
define epuck_IR ranger
(
	# define the pose of each transducer [xpos ypos heading]
	epuckIR( pose [0.030 -0.010 0 -15])	#fr right
	epuckIR( pose [0.022 -0.025 0 -45])	#fr right 2
	epuckIR( pose [0.0 -0.031 0 -90])		# right	
	epuckIR( pose [-0.03 -0.015 0 -150])	#back right
	epuckIR( pose [-0.03 0.015 0 150])		#back left
	epuckIR( pose [0.0 0.031 0 90])			#left
	epuckIR( pose [0.022 0.025 0 45])		#fr left 2
	epuckIR( pose [0.03 0.01 0 15])			#fr left
)

# epuck blobfinder
define epuck_cam blobfinder
(
		# number of colours to look for
		colors_count 1
	
		# which colours to look for
		colors ["yellow"]
	
		# camera parameters
		image [640 480]		#resolution
		
		size [0.001 0.001 0.001]
)


# epuck base
define epuck position
(
	# actual size
	size [0.07 0.07 0.12]
	
	# estimated mass in KG
	mass 0.5
	
	#Defining the shape of an epuck
	block(
		points 8
			point[0] [0.25 0]
			point[1] [0.75 0]
			point[2] [1 0.25]
			point[3] [1 0.75]
			point[4] [0.75 1]
			point[5] [0.25 1]
			point[6] [0 0.75]
			point[7] [0 0.25]
			z [0 1]
	)
	
	gui_outline 1
#	show_blocks 0

	# differential steering model
	drive "diff"

	obstacle_return 0
	fiducial_return 1
	ranger_return -1

	ctrl "candidatesolution"

	# sensors attached to epuck
	epuck_IR()  
	epuck_cam()
) 

define eAgent position
(
	# actual size
	size [0.07 0.07 0.12]
	
	# estimated mass in KG
	mass 0.5
	
	#Defining the shape of an epuck
	block(
		points 8
			point[0] [0.25 0]
			point[1] [0.75 0]
			point[2] [1 0.25]
			point[3] [1 0.75]
			point[4] [0.75 1]
			point[5] [0.25 1]
			point[6] [0 0.75]
			point[7] [0 0.25]
			z [0 1]
	)
	
	gui_outline 1
#	show_blocks 0

	# differential steering model
	drive "diff"

	obstacle_return 0
	
	ctrl "fateagent"
	
	ranger_return -1
)
