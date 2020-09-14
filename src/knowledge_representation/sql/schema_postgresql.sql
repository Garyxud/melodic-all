DROP SCHEMA public CASCADE;
CREATE SCHEMA public;
GRANT ALL ON SCHEMA public TO postgres;
GRANT ALL ON SCHEMA public TO public;

CREATE TABLE entities
(
    entity_id SERIAL NOT NULL,
    PRIMARY KEY (entity_id)
);

CREATE TYPE attribute_type as ENUM ('id', 'bool', 'int', 'float', 'str');

CREATE TABLE attributes
(
    attribute_name varchar(24)    NOT NULL,
    type           attribute_type NOT NULL,
    PRIMARY KEY (attribute_name)
);

CREATE TABLE concepts
(
    entity_id int NOT NULL,
    concept_name varchar(24) NOT NULL UNIQUE,
    PRIMARY KEY (entity_id, concept_name),
        FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE instance_of
(
    entity_id int NOT NULL,
    concept_name varchar(24) NOT NULL,
    PRIMARY KEY (entity_id, concept_name),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (concept_name)
        REFERENCES concepts (concept_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/******************* ENTITY ATTRIBUTES */

CREATE TABLE entity_attributes_id
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value int         NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_value)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_int
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value int         NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_str
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value varchar(24) NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_float
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value double precision NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_bool
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value bool,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/******************* MAPS */

CREATE TABLE maps
(
    entity_id int UNIQUE NOT NULL,
    map_id SERIAL NOT NULL ,
    map_name varchar(24) NOT NULL UNIQUE,
    PRIMARY KEY (map_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/* Deleting a map will delete owned entries in the geometry tables via cascade
   on their parent_map_id references, but their deletion doesn't cascade to the entities table.
   This trigger manually deletes entities associated with the map. */
CREATE FUNCTION delete_map_owned_entities() RETURNS TRIGGER AS $_$
BEGIN
    DELETE FROM entities WHERE entity_id IN (SELECT entity_id FROM (SELECT entity_id, parent_map_id FROM poses UNION SELECT entity_id, parent_map_id FROM points UNION SELECT entity_id, parent_map_id FROM regions) AS owned_entities WHERE parent_map_id = OLD.map_id);
    RETURN OLD;
END $_$ LANGUAGE 'plpgsql';

CREATE TRIGGER delete_map_owned_entities
BEFORE DELETE ON maps
FOR EACH ROW
EXECUTE PROCEDURE delete_map_owned_entities();

CREATE TABLE points
(
    entity_id int NOT NULL,
    point_name varchar(24) NOT NULL,
    parent_map_id int NOT NULL,
    point point NOT NULL,
    PRIMARY KEY (point_name, parent_map_id),
    UNIQUE (point_name, parent_map_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (parent_map_id)
        REFERENCES maps (map_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE regions
(
    entity_id int NOT NULL,
    region_name varchar(24) NOT NULL,
    parent_map_id int NOT NULL,
    region polygon NOT NULL,
    PRIMARY KEY (entity_id, region_name, parent_map_id),
    UNIQUE (region_name, parent_map_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (parent_map_id)
        REFERENCES maps (map_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE poses
(
    entity_id int NOT NULL,
    pose_name varchar(24) NOT NULL,
    parent_map_id int NOT NULL,
    pose lseg NOT NULL,
    PRIMARY KEY (entity_id, pose_name, parent_map_id),
    UNIQUE (pose_name, parent_map_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (parent_map_id)
        REFERENCES maps (map_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/* This doesn't seem to work because NEW isn't available in the subquery for the INSERT section */
CREATE FUNCTION normalize_pose() RETURNS TRIGGER AS $_$
BEGIN
    IF TG_OP = 'UPDATE' THEN
    WITH p (x1, y1, x2, y2) AS
    (SELECT st[0] AS x1, st[1] AS y1, en[0] as x2, en[1] AS y2 FROM
    (SELECT l[0] AS st, l[1] AS en FROM OLD.pose AS l) AS dummy),
    a(theta) AS (SELECT ATAN2(p.y2-p.y1,p.x2-p.x1) FROM p)
    SELECT lseg(point(p.x1, p.y1), point(p.x1 + COS(a.theta), p.y1 + SIN(a.theta))) INTO OLD.pose FROM p;
    RETURN OLD;
    ELSIF TG_OP = 'INSERT' THEN
    SELECT lseg(point(x1, y1), point(x1 + COS(theta), y1 + SIN(theta))) INTO NEW.pose FROM
    (SELECT st[0] AS x1, st[1] AS y1, en[0] AS x2, en[1] AS y2, ATAN2(y2-y1,x2-x1) AS theta FROM
    (SELECT l[0] AS st, l[1] AS en FROM NEW.pose) AS l) AS dummy;
    RETURN NEW;
    ELSE
    RETURN null;
    END IF;

END $_$ LANGUAGE 'plpgsql';

/*CREATE TRIGGER normalize_pose
BEFORE UPDATE OF pose OR INSERT ON poses
FOR EACH ROW
EXECUTE PROCEDURE normalize_pose();*/

/* Simplifies retrieving poses as (x,y,theta) tuples*/
CREATE VIEW poses_point_angle AS
  SELECT entity_id, start[0] as x, start[1] as y, ATAN2(end_point[1]-start[1],end_point[0]-start[0])
                 as theta, pose_name, parent_map_id FROM (SELECT entity_id, pose[0] as start, pose[1] as end_point, pose_name, parent_map_id FROM
                poses) AS dummy_sub_alias;

CREATE TABLE doors
(
    entity_id int NOT NULL,
    door_name varchar(24) NOT NULL,
    parent_map_id int NOT NULL,
    door lseg NOT NULL,
    PRIMARY KEY (entity_id, door_name, parent_map_id),
    UNIQUE (door_name, parent_map_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (parent_map_id)
        REFERENCES maps (map_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/******************* FUNCTIONS */

CREATE FUNCTION remove_attribute(INT, varchar(24))
    RETURNS BIGINT
    LANGUAGE plpgsql
AS
$body$
DECLARE
    n_id_del    bigint;
    n_bool_del  bigint;
    n_int_del    bigint;
    n_float_del bigint;
    n_str_del   bigint;
BEGIN

    WITH id_del AS (DELETE FROM entity_attributes_id WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM id_del
    INTO n_id_del;

    WITH bool_del
             AS (DELETE FROM entity_attributes_bool WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM bool_del
    INTO n_bool_del;

    WITH int_del AS (DELETE FROM entity_attributes_int WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM int_del
    INTO n_int_del;

    WITH float_del AS (DELETE FROM entity_attributes_float WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM float_del
    INTO n_float_del;

    WITH str_del AS (DELETE FROM entity_attributes_str WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM str_del
    INTO n_str_del;
    RETURN n_id_del + n_bool_del + n_int_del + n_float_del + n_str_del;
END
$body$;

CREATE FUNCTION get_concepts_recursive(INT)
    RETURNS TABLE
            (
                entity_id INT,
                concept_name varchar(24)
            )
    IMMUTABLE
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (ID)
   AS
   (
       /* Get whatever the argument is an instance of, and then every thing that that is a descended concept of*/
       SELECT concepts.entity_id
       FROM concepts
       INNER JOIN instance_of ON (instance_of.entity_id = $1 AND instance_of.concept_name = concepts.concept_name)

       UNION ALL

       SELECT a.attribute_value
       FROM entity_attributes_id a
                INNER JOIN cteConcepts b
                           ON a.attribute_name = 'is_a'
                               AND a.entity_id = b.ID
   )
SELECT entity_id, concept_name
FROM cteConcepts INNER JOIN concepts ON entity_id = ID;
$$;

CREATE FUNCTION get_all_concept_ancestors(INT)
    RETURNS TABLE
            (
                entity_id INT,
                concept_name varchar(24)
            )
    IMMUTABLE
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (id)
   AS
   (
       /* Make sure the argument is a concept */
       SELECT entity_id FROM concepts WHERE (entity_id = $1)
       UNION ALL
       SELECT a.attribute_value
       FROM entity_attributes_id a
                INNER JOIN cteConcepts b
                           ON a.attribute_name = 'is_a'
                               AND a.entity_id = b.ID
   )
SELECT entity_id, concept_name
FROM cteConcepts INNER JOIN concepts ON entity_id = ID;
$$;


CREATE FUNCTION get_all_concept_descendants(INT)
    RETURNS TABLE
            (
                entity_id INT,
                concept_name varchar(24)
            )
    IMMUTABLE
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (id)
AS
(
   /* Make sure the argument is a concept */
   SELECT entity_id FROM concepts WHERE (entity_id = $1)
   UNION ALL
   SELECT a.entity_id
   FROM entity_attributes_id a
            INNER JOIN cteConcepts b
                       ON a.attribute_name = 'is_a'
                        AND a.attribute_value = b.ID
)
SELECT entity_id, concept_name
FROM cteConcepts INNER JOIN concepts ON entity_id = ID;
$$;


CREATE FUNCTION get_all_instances_of_concept_recursive(INT)
    RETURNS TABLE
            (
                entity_id INT,
                concept_name varchar(24)
            )
    IMMUTABLE
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (id)
AS
(
   /* Make sure the argument is a concept */
   SELECT entity_id FROM concepts WHERE (entity_id = $1)
   UNION ALL
   SELECT a.entity_id
   FROM entity_attributes_id a
            INNER JOIN cteConcepts b
                       ON a.attribute_name = 'is_a'
                        AND a.attribute_value = b.ID
)
SELECT entity_id, concept_name
FROM instance_of WHERE concept_name IN (SELECT concept_name FROM cteConcepts INNER JOIN concepts ON (concepts.entity_id = id));
$$;

/***** DEFAULT VALUES */
CREATE FUNCTION add_default_attributes()
    RETURNS VOID
    LANGUAGE SQL
AS
$$

INSERT INTO attributes
VALUES ('answer_to', 'id'),
('count', 'int'),
('default_location', 'id'),
('has', 'id'),
('height', 'float'),
('is_a', 'id'),
('is_connected', 'id'),
('is_delivered', 'id'),
('is_facing', 'id'),
('is_holding', 'id'),
('is_in', 'id'),
('is_near', 'id'),
('is_open', 'bool'),
('is_placed', 'id'),
('name', 'str'),
('part_of', 'id');
$$;


CREATE FUNCTION add_default_entities()
    RETURNS bigint
    LANGUAGE SQL
AS
$$

INSERT INTO entities
VALUES (1), (2), (3), (4), (5), (6);
INSERT INTO concepts
VALUES (2, 'robot'), (3, 'map'), (4, 'point'), (5, 'pose'), (6, 'region') ;
INSERT INTO instance_of
VALUES (1, 'robot');

/* Manual inserts will mess up the SERIAL sequence, so we have to manually bump the number*/
SELECT setval('entities_entity_id_seq', max(entity_id))
FROM   entities;
$$;

SELECT *
FROM add_default_attributes();
SELECT *
FROM add_default_entities();
