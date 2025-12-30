"""
Face Storage - Persistence for face encodings and training images

Storage structure:
    ~/.hexapod/face_recognition/
    ├── encodings.pkl          # Serialized face encodings
    └── training_images/
        ├── alice/
        │   ├── 001.jpg
        │   └── 002.jpg
        └── bob/
            └── 001.jpg
"""

import os
import pickle
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import numpy as np

# OpenCV for image I/O
try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False


class FaceStorage:
    """Manages face encoding persistence and training image storage"""

    def __init__(self, base_path: str):
        """
        Initialize face storage.

        Args:
            base_path: Base directory for face data (e.g., ~/.hexapod/face_recognition)
        """
        self.base_path = Path(os.path.expanduser(base_path))
        self.encodings_file = self.base_path / 'encodings.pkl'
        self.images_dir = self.base_path / 'training_images'

        # Ensure directories exist
        self.base_path.mkdir(parents=True, exist_ok=True)
        self.images_dir.mkdir(exist_ok=True)

    def load_encodings(self) -> Dict[str, List[np.ndarray]]:
        """
        Load face encodings from disk.

        Returns:
            Dictionary mapping person names to lists of face encodings
            (each encoding is a 128-dimensional numpy array)
        """
        if self.encodings_file.exists():
            try:
                with open(self.encodings_file, 'rb') as f:
                    return pickle.load(f)
            except Exception as e:
                print(f'Warning: Failed to load encodings: {e}')
        return {}

    def save_encodings(self, encodings: Dict[str, List[np.ndarray]]) -> bool:
        """
        Save face encodings to disk.

        Args:
            encodings: Dictionary mapping person names to encoding lists

        Returns:
            True if successful, False otherwise
        """
        try:
            with open(self.encodings_file, 'wb') as f:
                pickle.dump(encodings, f)
            return True
        except Exception as e:
            print(f'Error saving encodings: {e}')
            return False

    def save_training_image(
        self,
        person_name: str,
        image: np.ndarray,
        face_location: Optional[Tuple[int, int, int, int]] = None
    ) -> Tuple[bool, str, int]:
        """
        Save a training image for a person.

        Args:
            person_name: Name of the person
            image: RGB image as numpy array
            face_location: Optional (top, right, bottom, left) to crop face

        Returns:
            Tuple of (success, message, total_images_for_person)
        """
        if not OPENCV_AVAILABLE:
            return False, 'OpenCV not available for image saving', 0

        # Sanitize person name (alphanumeric and underscores only)
        safe_name = ''.join(c if c.isalnum() or c == '_' else '_' for c in person_name)
        if not safe_name:
            return False, 'Invalid person name', 0

        person_dir = self.images_dir / safe_name
        person_dir.mkdir(exist_ok=True)

        # Find next image number
        existing = list(person_dir.glob('*.jpg'))
        next_num = len(existing) + 1

        # Crop face if location provided
        if face_location:
            top, right, bottom, left = face_location
            # Add some padding
            padding = 20
            top = max(0, top - padding)
            left = max(0, left - padding)
            bottom = min(image.shape[0], bottom + padding)
            right = min(image.shape[1], right + padding)
            image = image[top:bottom, left:right]

        # Convert RGB to BGR for OpenCV
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Save image
        image_path = person_dir / f'{next_num:03d}.jpg'
        try:
            cv2.imwrite(str(image_path), image_bgr)
            return True, f'Saved {image_path.name}', next_num
        except Exception as e:
            return False, f'Failed to save image: {e}', len(existing)

    def get_training_images(
        self,
        person_name: Optional[str] = None
    ) -> Dict[str, List[str]]:
        """
        Get paths to training images.

        Args:
            person_name: Optional filter for specific person

        Returns:
            Dictionary mapping person names to lists of image paths
        """
        result = {}

        if person_name:
            # Sanitize name
            safe_name = ''.join(
                c if c.isalnum() or c == '_' else '_' for c in person_name
            )
            person_dir = self.images_dir / safe_name
            if person_dir.exists():
                result[safe_name] = sorted(
                    str(p) for p in person_dir.glob('*.jpg')
                )
        else:
            # Get all persons
            for person_dir in sorted(self.images_dir.iterdir()):
                if person_dir.is_dir():
                    images = sorted(str(p) for p in person_dir.glob('*.jpg'))
                    if images:
                        result[person_dir.name] = images

        return result

    def list_identities(self) -> List[Tuple[str, int]]:
        """
        List all identities and their image counts.

        Returns:
            List of (name, image_count) tuples
        """
        images = self.get_training_images()
        return [(name, len(paths)) for name, paths in sorted(images.items())]

    def delete_person(self, person_name: str) -> Tuple[bool, str]:
        """
        Delete all training data for a person.

        Args:
            person_name: Name of person to delete

        Returns:
            Tuple of (success, message)
        """
        import shutil

        safe_name = ''.join(
            c if c.isalnum() or c == '_' else '_' for c in person_name
        )
        person_dir = self.images_dir / safe_name

        if not person_dir.exists():
            return False, f'Person {person_name} not found'

        try:
            shutil.rmtree(person_dir)

            # Also remove from encodings if present
            encodings = self.load_encodings()
            if safe_name in encodings:
                del encodings[safe_name]
                self.save_encodings(encodings)

            return True, f'Deleted all data for {person_name}'
        except Exception as e:
            return False, f'Failed to delete: {e}'

    def get_storage_info(self) -> Dict:
        """
        Get storage statistics.

        Returns:
            Dictionary with storage information
        """
        identities = self.list_identities()
        encodings = self.load_encodings()

        total_images = sum(count for _, count in identities)
        total_encodings = sum(len(encs) for encs in encodings.values())

        return {
            'base_path': str(self.base_path),
            'num_identities': len(identities),
            'total_images': total_images,
            'total_encodings': total_encodings,
            'identities': identities,
        }
