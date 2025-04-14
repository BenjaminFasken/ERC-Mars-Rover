sudo apt-get install unzip


for f in marsYardData/*.zip; do
    folder_name="${f%.zip}"  # Remove .zip extension
    echo "Unzipping $f into $folder_name..."
    mkdir -p "$folder_name"
    unzip -o "$f" -d "$folder_name"
done
