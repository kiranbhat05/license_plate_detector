import json
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.backends import default_backend
from datetime import datetime

# Metadata class for storing information
class Metadata:
    """Class to hold metadata information like time, date, number plate, GNSS data, and automobile ID."""
    def __init__(self, number_plate, gnss_data, automobile_id):
        self.automobile_id = automobile_id  # automobile ID
        self.date_time = datetime.now().isoformat()  # Serialize date-time as string
        self.number_plate = number_plate
        self.gnss_data = gnss_data  # GNSS data

def load_public_key(filename):
    """Load RSA public key from a file."""
    with open(filename, "rb") as key_file:
        public_key = serialization.load_pem_public_key(
            key_file.read(),
            backend=default_backend()
        )
    return public_key

def encrypt_data(data, public_key):
    """Encrypt data using the RSA public key."""
    encrypted_data = public_key.encrypt(
        data.encode(),  # Convert the string to bytes
        padding.OAEP(
            mgf=padding.MGF1(algorithm=hashes.SHA256()),
            algorithm=hashes.SHA256(),
            label=None
        )
    )
    return encrypted_data

def main():
    # Load the public key
    public_key = load_public_key("public_key.pem")

    # Create a Metadata object
    metadata = Metadata(
        number_plate="ABC1234",
        gnss_data={"latitude": 51.5074, "longitude": 0.1278},  # Example GNSS data
        automobile_id=1234
    )

    # Serialize the Metadata object to JSON
    metadata_json = json.dumps(metadata.__dict__)
    print(f"Serialized Metadata: {metadata_json}")

    # Encrypt the serialized data
    encrypted_metadata = encrypt_data(metadata_json, public_key)

    # Save the encrypted data to a file
    with open("encrypted_metadata.txt", "wb") as file:
        file.write(encrypted_metadata)

    print("Metadata encrypted and saved to 'encrypted_metadata.txt'.")

if __name__ == "__main__":
    main()
