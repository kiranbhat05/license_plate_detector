import json
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.backends import default_backend

class Metadata:
    """Class to hold metadata information like time, date, number plate, GNSS data, and automobile ID."""
    def __init__(self, number_plate, gnss_data, automobile_id):
        self.automobile_id = automobile_id
        self.date_time = None
        self.number_plate = number_plate
        self.gnss_data = gnss_data

def load_private_key(filename):
    """Load RSA private key from a file."""
    with open(filename, "rb") as key_file:
        private_key = serialization.load_pem_private_key(
            key_file.read(),
            password=None,
            backend=default_backend()
        )
    return private_key

def decrypt_data(encrypted_data, private_key):
    """Decrypt the encrypted data using RSA private key."""
    decrypted_data = private_key.decrypt(
        encrypted_data,
        padding.OAEP(
            mgf=padding.MGF1(algorithm=hashes.SHA256()),
            algorithm=hashes.SHA256(),
            label=None
        )
    )
    return decrypted_data.decode()

def main():
    # Load the private key
    private_key = load_private_key("private_key.pem")

    # Read the encrypted metadata from the file
    with open("encrypted_metadata.txt", "rb") as file:
        encrypted_metadata = file.read()

    # Decrypt the metadata
    decrypted_metadata_json = decrypt_data(encrypted_metadata, private_key)
    print(f"Decrypted Metadata JSON: {decrypted_metadata_json}")

    # Deserialize the JSON back into a Metadata object
    decrypted_metadata_dict = json.loads(decrypted_metadata_json)
    print(f"Deserialized Metadata: {decrypted_metadata_dict}")

if __name__ == "__main__":
    main()
