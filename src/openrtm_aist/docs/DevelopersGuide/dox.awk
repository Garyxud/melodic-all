{
  printf("mv %s %s.org\n", $1, $1);
  printf("sed -e \'s/includegraphics\\[\\([a-z\\=0-9]*\\)\\]\{/includegraphics\\[\\1\\]\{\\\\doxygendir\\//g\' %s.org > %s\n", $1, $1);
  printf("rm %s.org\n", $1);
}
