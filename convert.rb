#!/usr/bin/env ruby

require 'net/http'
require 'net/http/post/multipart'
require 'json'

categories = {
  'urban outdoor' => [
    'dumpster',
    'fire_hydrant',
    'lamp_post',
    'mailbox',
    'construction_barrel',
    'construction_cone',
  ],

  'terrain' => [
    'asphalt_plane',
    'ground_plane',
    'mud_box',
  ],

  'container' => [
    'beer',
    'bowl',
    'coke_can',
    'saucepan',
    'plastic_cup'
  ],

  'building' => [
    'fast_food',
    'gas_station',
    'house_1',
    'house_2',
    'house_3',
    'iss',
    'iss_half',
    'kitchen_dining',
    'office_building',
    'powerplant',
    'starting_pen',
    'willowgarage'
  ],

  'furniture' => [
    'bookshelf',
    'cabinet',
    'table',
    'table_marble',
  ],

  'NIST' => [
    'nist_maze_wall_120',
    'nist_maze_wall_240',
    'nist_maze_wall_triple_holes_120',
    'nist_simple_ramp_120',
    'nist_stairs_120',
    'nist_elevated_floor_120',
    'nist_fiducial_barrel',
  ],

  'Robocup Soccer' => [
    'robocup09_spl_field',
    'robocup14_spl_field',
    'robocup_3Dsim_field',
    'robocup14_spl_goal',
    'robocup_3Dsim_goal',
    'robocup_spl_ball',
    'robocup_3Dsim_ball',
  ],

  'robot'=> [
    'create',
    'husky',
    'irobot_hand',
    'pioneer2dx',
    'pioneer3at',
    'pr2',
    'pr2_gripper',
    'quadrotor',
    'robonaut',
    'simple_arm',
    'simple_arm_gripper',
    'simple_gripper',
    'turtlebot',
    'youbot',
  ],

  'sensor' => [
    'camera',
    'stereo_camera',
    'hokuyo',
    'kinect',
  ],

  'sign' => [
    'speed_limit_sign',
    'stop_sign',
  ],

  'vehicle' => [
    'utility_cart',
    'polaris_ranger_ev',
    'polaris_ranger_xp900',
    'polaris_ranger_xp900_no_roll_cage',
    'cart_front_steer',
    'cart_rigid_suspension',
    'cart_soft_suspension',
  ],

  'tool' => [
    'cordless_drill',
    'hammer',
    'monkey_wrench',
  ],

  'DARPA Robotics Challenge' => [
    'drc_practice_base_4x8',
    'drc_practice_truss',
    'drc_practice_yellow_parking_block',
    'drc_practice_orange_jersey_barrier',
    'drc_practice_white_jersey_barrier',
    'drc_practice_angled_barrier_45',
    'drc_practice_angled_barrier_135',
    'drc_practice_ball_valve',
    'drc_practice_handle_wheel_valve',
    'drc_practice_hand_wheel_valve',
    'drc_practice_wheel_valve',
    'drc_practice_wheel_valve_large',
    'drc_practice_2x4',
    'drc_practice_2x6',
    'drc_practice_4x4x20',
    'drc_practice_4x4x40',
    'drc_practice_blue_cylinder',
    'drc_practice_wood_slats',
    'drc_practice_ball_valve_wall',
    'drc_practice_handle_wheel_valve_wall',
    'drc_practice_hand_wheel_valve_wall',
    'drc_practice_valve_wall',
    'drc_practice_wheel_valve_wall',
    'drc_practice_wheel_valve_large_wall',
    'drc_practice_door_4x8',
    'drc_practice_ladder',
    'drc_practice_block_wall',
  ],

  'wall' => [
    'brick_box_3x1x3',
    'jersey_barrier',
    'grey_wall',
  ],

  'building material' => [
    'cinder_block',
    'cinder_block_2',
    'cinder_block_wide',
    'door_handle',
    'hinged_door',
    'stone_10_2_5_1cm',
    'standpipe',
    'tube_2_25cm',
    'tube_9_5mm',
    'washer',
    'wood_block_10_2_1cm',
    'wood_cube_10cm',
    'wood_cube_2_5cm',
    'wood_cube_5cm',
    'wood_cube_7_5cm',
    'wooden_peg',
  ],

  'miscellaneous' => [
    'fire_hose_long',
    'fire_hose_long_curled',

    'double_pendulum_with_base',
    'breakable_test',
    'cube_20k',
    'ball_bearing',
    'cricket_ball',
    'marble_1_5cm',
    'metal_peg',
    'sun',
    'ragdoll',
  ]
}

url = URI.parse('http://thepropshop.org/api/upload')

Dir.glob("*/").each do |dir|
  dir = dir.to_s.sub(/\//, '')
  categories.each_pair do |key, value|
    if value.include? dir.to_s
      configFile = File.join(dir,'model.config')
      sdfFile = File.absolute_path(File.join(dir, 'model.sdf'))
      content = File.read(configFile)

      model_desc = content.match(/<description>(.*)<\/description>/m)
      if !model_desc.nil?
        model_desc = model_desc[1].strip
      else
        model_desc = "No description"
      end

      model_name = content.match(/<name>(.*)<\/name>/)[1].strip
      `gzprop #{sdfFile}`

      req = Net::HTTP::Post::Multipart.new url.path, 
          {'model'=>{'name'=> model_name, 'description' => model_desc,
                     'category'=>key}, 'user'=>'5412397770726f500f000000',
                     'file'=>UploadIO.new(File.new("#{dir}.zip"),
                                          "application/zip", "#{dir}.zip")}

      res = Net::HTTP.start(url.host, url.port) do |http|
        http.request(req)
      end
      sleep(2.0)

      #content = File.read(configFile).gsub(/<\/model>/, "  <category>#{key}</category>\n</model>")
      #File.open(configFile, 'wb') { |file| file.write(content)}
    end
  end
end
