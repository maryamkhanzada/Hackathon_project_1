#!/usr/bin/env python3
"""
Multi-Modal VLA System
Integrates Vision, Language, and Action for robotic control
Combines Whisper (voice), CLIP (vision), and GPT-4 (reasoning)
"""

import whisper
import clip
import torch
import openai
import cv2
import numpy as np
from PIL import Image
from typing import Dict, List, Optional, Tuple


class MultiModalVLASystem:
    """
    Complete Vision-Language-Action system for robots
    Fuses voice commands with visual perception for grounded actions
    """

    def __init__(self, openai_key: str, whisper_model="small", clip_model="ViT-B/32"):
        """
        Initialize multi-modal VLA system

        Args:
            openai_key: OpenAI API key for GPT-4
            whisper_model: Whisper model size
            clip_model: CLIP model variant
        """
        # Initialize models
        print("Loading Whisper for speech recognition...")
        self.whisper = whisper.load_model(whisper_model)

        print("Loading CLIP for visual grounding...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.clip_model, self.clip_preprocess = clip.load(clip_model, device=self.device)

        print("Setting up GPT-4 for task planning...")
        openai.api_key = openai_key

        print("Multi-Modal VLA System ready!")

    def process_voice_command(self, audio_data: np.ndarray, image: np.ndarray) -> Dict:
        """
        Process voice command with visual context

        Args:
            audio_data: Audio samples (float32, normalized)
            image: RGB image (numpy array)

        Returns:
            Dictionary with grounded action plan
        """

        # Step 1: Transcribe voice command
        transcript = self.transcribe_audio(audio_data)
        print(f"Transcribed: \"{transcript}\"")

        if not transcript.strip():
            return {"error": "No speech detected"}

        # Step 2: Ground language in visual scene
        grounded_objects = self.ground_visual_references(transcript, image)
        print(f"Grounded objects: {grounded_objects}")

        # Step 3: Generate action plan with GPT-4
        action_plan = self.generate_action_plan(transcript, grounded_objects)
        print(f"Action plan: {action_plan}")

        return {
            "transcript": transcript,
            "grounded_objects": grounded_objects,
            "action_plan": action_plan
        }

    def transcribe_audio(self, audio_data: np.ndarray) -> str:
        """Transcribe audio with Whisper"""
        result = self.whisper.transcribe(audio_data, fp16=False)
        return result["text"].strip()

    def ground_visual_references(self, text: str, image: np.ndarray) -> List[Dict]:
        """
        Ground language references (e.g., "red cup", "that object") in image

        Args:
            text: Natural language text
            image: RGB image

        Returns:
            List of grounded objects with bounding boxes
        """

        # Extract noun phrases (simplified - would use spaCy in production)
        potential_objects = self.extract_object_references(text)

        if not potential_objects:
            return []

        # Use CLIP to find objects in image
        grounded = []

        for obj_desc in potential_objects:
            bbox, confidence = self.clip_ground_object(image, obj_desc)

            if bbox is not None:
                grounded.append({
                    "description": obj_desc,
                    "bbox": bbox,
                    "confidence": confidence
                })

        return grounded

    def extract_object_references(self, text: str) -> List[str]:
        """
        Extract object references from text (simplified)

        In production, use spaCy or similar for proper NER/chunking
        """

        # Simple keyword matching (replace with spaCy in production)
        objects = []

        object_keywords = [
            "cup", "glass", "plate", "bowl", "bottle", "can",
            "phone", "remote", "book", "laptop", "chair", "table"
        ]

        colors = ["red", "blue", "green", "yellow", "black", "white"]

        text_lower = text.lower()

        for obj in object_keywords:
            if obj in text_lower:
                # Check for color modifier
                for color in colors:
                    if color in text_lower and (text_lower.find(color) < text_lower.find(obj)):
                        objects.append(f"{color} {obj}")
                        break
                else:
                    objects.append(obj)

        return objects

    def clip_ground_object(self, image: np.ndarray, description: str) -> Tuple[Optional[List], float]:
        """
        Use CLIP to find object matching description in image

        Args:
            image: RGB image
            description: Text description of object

        Returns:
            (bounding_box, confidence) or (None, 0.0)
        """

        # For simplicity, we'll use sliding window approach
        # In production, combine with object detector (YOLO) for proposals

        h, w = image.shape[:2]
        best_score = -float('inf')
        best_bbox = None

        # Grid search (simplified - use object proposals in production)
        for scale in [0.2, 0.4, 0.6]:
            window_size = (int(h * scale), int(w * scale))

            for y in range(0, h - window_size[0], window_size[0] // 2):
                for x in range(0, w - window_size[1], window_size[1] // 2):
                    crop = image[y:y+window_size[0], x:x+window_size[1]]

                    # Compute CLIP similarity
                    score = self.compute_clip_similarity(crop, description)

                    if score > best_score:
                        best_score = score
                        best_bbox = [x, y, x + window_size[1], y + window_size[0]]

        if best_score > 0.2:  # Threshold
            return best_bbox, float(best_score)

        return None, 0.0

    def compute_clip_similarity(self, image_crop: np.ndarray, text: str) -> float:
        """Compute CLIP similarity between image and text"""

        # Preprocess image
        pil_image = Image.fromarray(cv2.cvtColor(image_crop, cv2.COLOR_BGR2RGB))
        image_input = self.clip_preprocess(pil_image).unsqueeze(0).to(self.device)

        # Tokenize text
        text_input = clip.tokenize([text]).to(self.device)

        # Compute embeddings
        with torch.no_grad():
            image_features = self.clip_model.encode_image(image_input)
            text_features = self.clip_model.encode_text(text_input)

            # Normalize
            image_features /= image_features.norm(dim=-1, keepdim=True)
            text_features /= text_features.norm(dim=-1, keepdim=True)

            # Cosine similarity
            similarity = (image_features @ text_features.T).item()

        return similarity

    def generate_action_plan(self, command: str, grounded_objects: List[Dict]) -> List[Dict]:
        """
        Generate action plan using GPT-4 with visual grounding

        Args:
            command: Natural language command
            grounded_objects: List of grounded objects from vision

        Returns:
            Action plan as list of steps
        """

        # Create prompt with visual context
        grounded_str = "\n".join(
            f"- {obj['description']} at bbox {obj['bbox']} (confidence: {obj['confidence']:.2f})"
            for obj in grounded_objects
        )

        prompt = f"""
You are a robot task planner with visual perception.

User command: "{command}"

Detected objects in scene:
{grounded_str if grounded_str else "No objects detected"}

Generate an action plan using these actions:
- navigate(x, y) - Move to position
- approach_object(bbox) - Approach object at bounding box
- grasp_object(bbox) - Grasp object
- place_object(x, y) - Place held object at position

Output JSON: {{"steps": [{{"action": "...", "params": {{...}}}}]}}
"""

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4-turbo-preview",
                messages=[
                    {"role": "system", "content": "You are a robot with vision and manipulation capabilities."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1
            )

            import json
            plan = json.loads(response.choices[0].message.content)
            return plan.get("steps", [])

        except Exception as e:
            print(f"Error generating plan: {e}")
            return []


# Example usage
if __name__ == "__main__":
    # Initialize system
    vla = MultiModalVLASystem(
        openai_key="your_openai_key_here",
        whisper_model="small",
        clip_model="ViT-B/32"
    )

    # Simulate voice command with image
    # In real system, capture from microphone and camera

    # Load example image
    image = cv2.imread("example_scene.jpg")

    # Simulate audio (in real system, record from microphone)
    # For testing, transcribe from text
    test_audio = np.zeros(16000 * 3, dtype=np.float32)  # 3 seconds of silence

    print("Processing command: 'Pick up the red cup'")

    result = vla.process_voice_command(test_audio, image)

    print("\nResult:")
    print(f"  Transcript: {result.get('transcript', 'N/A')}")
    print(f"  Grounded objects: {len(result.get('grounded_objects', []))}")
    print(f"  Action plan: {result.get('action_plan', [])}")
