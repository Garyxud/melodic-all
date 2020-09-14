#!/usr/bin/env bash

sudo service postgresql start

# Set a fixed password for CI
if [[ -n "$IN_DOCKER" ]]; then
  ENTERED_PASSWORD="nopass"
else
  ENTERED_PASSWORD="$1"
fi

if [[ -z "$ENTERED_PASSWORD" ]]; then
  echo "Enter a password that will be used for local connections to the database. This will be stored in plaintext in the current user account."
  read -sp 'Password: ' ENTERED_PASSWORD
fi

schema_path="$(rospack find knowledge_representation)/sql/schema_postgresql.sql"

sudo -u postgres createdb knowledge_base
sudo -u postgres psql -d knowledge_base -f $schema_path
sudo -u postgres psql -c "ALTER USER postgres WITH PASSWORD '$ENTERED_PASSWORD'"

# Store the password in a dotfile so we can avoid authentication elsewhere
cat > ~/.pgpass <<EOF
# hostname:port:database:username:password
localhost:*:knowledge_base:postgres:$ENTERED_PASSWORD
EOF

chmod 600 ~/.pgpass

if [[ -n "$IN_DOCKER" ]]; then
  echo "Executing Industrial CI setup"
  # The previous invocations won't work because the postgres user doesn't
  # have permission to access the working directory
  POSTGRES_HOME=/var/lib/postgresql
  cp sql/schema_postgresql.sql $POSTGRES_HOME
  sudo -u postgres psql -d knowledge_base -f $POSTGRES_HOME/schema_postgresql.sql
fi