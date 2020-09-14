#!/usr/bin/env bash

ikr() {
  rosrun knowledge_representation ikr $@
}

kr-show() {
    rosrun knowledge_representation show_me $@
}
# Is there a default MySQL knowledge_base set up?
if mysql -u root -u root -h localhost knowledge_base  -e "SELECT * FROM entities LIMIT 1" &> /dev/null; then
MYSQL_FOUND="true"
kr-save() {
    stamp=$(date +%Y-%m-%d_%H-%M-%S)
    if [[ $# == 1 ]]; then
       stamp=$1
    fi
    mysqldump -u root -p --databases knowledge_base > "knowledge_${stamp}.sql"
}
fi

# NOTE: If you have both mysql and postgres installed, the Postgres helpers take precedence
# Is there a default Postgres knowledge_base set up?
if  psql -U postgres --host=localhost -d knowledge_base -c "SELECT * FROM entities LIMIT 1" &> /dev/null; then
if [[ "$MYSQL_FOUND" == "true" ]]; then
  echo "Both MySQL and Postgres knowledge bases are initialized. Shell helpers will use Postgres"
fi
kr-query() {
    psql -U postgres --host=localhost -d knowledge_base -c "$1"
}

kr-save() {
    stamp=$(date +%Y-%m-%d_%H-%M-%S)
    if [[ $# == 1 ]]; then
       stamp=$1
    fi
  pg_dump -U postgres --host=localhost knowledge_base > "knowledge_${stamp}.sql"
}
fi

