apt-get install libssl-dev -y
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
make
make install
cd ..

git clone https://github.com/eclipse/paho.mqtt.cpp
cd paho.mqtt.cpp
make
make install
cd ..
