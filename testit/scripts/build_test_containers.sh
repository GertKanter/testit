QUERY=$(docker images -q testitros/testit_patrol:latest)
if [ ${#QUERY} -eq 0 ]; then
  echo "Building testit container..."
  cd $(rospack find testit)/docker/testit
  docker build --no-cache -t testitros/testit_patrol:latest .
fi

QUERY=$(docker images -q testitros/sut_patrol:latest)
if [ ${#QUERY} -eq 0 ]; then
  echo "Building sut container..."
  cd $(rospack find testit)/docker/sut
  docker build --no-cache -t testitros/sut_patrol:latest .
fi
