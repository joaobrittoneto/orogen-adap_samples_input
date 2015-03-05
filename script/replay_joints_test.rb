#!/usr/bin/ruby1.9.1
require 'vizkit'
require 'orocos'

Orocos::CORBA.max_message_size = 80000000

Orocos.initialize

Orocos.load_typekit_for '/base/samples/Joints'
type = Orocos.registry.get '/base/samples/Joints'
puts type

if ARGV.size < 1
    puts "usage: log-files"
    exit(0)
end

replay = Orocos::Log::Replay.open(ARGV)

Vizkit.control replay
Vizkit.exec


