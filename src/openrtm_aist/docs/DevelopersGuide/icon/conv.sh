ls *png | awk '{gsub(".png",""); printf("bmeps -c %s.png %s.eps\n", $1, $1);}' |sh
