while IFS= read -r line; do
  echo -e "\033[32m[$1]\033[0m $line"
done
