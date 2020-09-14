#!/usr/bin/env bash

# If the script breaks:
# 1. Make sure that MySQL is installed. Don't use your distribution's package. Get the latest `.deb` from [the MySQL website](https://dev.mysql.com/doc/mysql-apt-repo-quick-guide/en/#apt-repo-fresh-install).
# 2. Next [clone MySQL Connector/C++ Version 8](https://dev.mysql.com/downloads/connector/cpp/) with
#        git clone https://github.com/mysql/mysql-connector-cpp.git
#    Then build:

#       mkdir bulid
#       cd build
#       cmake ..
#       sudo make -j2 install

#3. You will also need MySQL Shell installed. Directions [here](https://dev.mysql.com/doc/refman/5.7/en/installing-mysql-shell-linux-quick.html).
# 4. Get the x plugin running with [these directions](https://dev.mysql.com/doc/refman/5.7/en/document-store-setting-up.html).

# Set selections in otherwise annoying interactive deb config
debconf-set-selections <<< 'mysql-apt-config mysql-apt-config/repo-codename select trusty'
debconf-set-selections <<< 'mysql-apt-config mysql-apt-config/repo-distro select ubuntu'
debconf-set-selections <<< 'mysql-apt-config mysql-apt-config/repo-url string http://repo.mysql.com/apt/'
debconf-set-selections <<< 'mysql-apt-config mysql-apt-config/select-preview select '
debconf-set-selections <<< 'mysql-apt-config mysql-apt-config/select-product select Ok'
debconf-set-selections <<< 'mysql-apt-config mysql-apt-config/select-server select mysql-8.0'
debconf-set-selections <<< 'mysql-apt-config mysql-apt-config/select-tools select '
debconf-set-selections <<< 'mysql-apt-config mysql-apt-config/unsupported-platform select abort'

# Set a fixed password for CI
if [[ -n "$IN_DOCKER" ]]; then
  sudo apt install git -y
fi

cd /tmp && \
sudo apt install wget && \
wget https://dev.mysql.com/get/mysql-apt-config_0.8.12-1_all.deb -O sql.deb && \
sudo DEBIAN_FRONTEND=noninteractive dpkg -i sql.deb && \
sudo apt update && \
sudo apt install -y mysql-server &&\
git clone https://github.com/mysql/mysql-connector-cpp.git &&\
cd mysql-connector-cpp &&\
git checkout tags/8.0.15 &&\
mkdir build &&\
cd build &&\
cmake .. &&\
sudo make -j4 install &&\
sudo mv libmysqlcppconn* /usr/lib/x86_64-linux-gnu/ &&\
sudo mysql -e "ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY ''" &&\
sudo mysql -u root -p -e "INSTALL PLUGIN mysqlx SONAME 'mysqlx.so';" &&\
roscd knowledge_representation &&\
mysql -u root -p -e "source sql/schema_mysql.sql"
