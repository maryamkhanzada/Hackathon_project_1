#!/usr/bin/env python3
"""
LLM Task Planner for Robotic Systems
Integrates with OpenAI GPT-4 or local LLMs for task decomposition
"""

import openai
import json
from typing import List, Dict, Optional

class LLMTaskPlanner:
    """
    Task planner using Large Language Models
    Breaks down high-level commands into executable robot actions
    """

    def __init__(self, api_key: str, model: str = "gpt-4-turbo-preview"):
        """
        Initialize LLM planner

        Args:
            api_key: OpenAI API key
            model: Model name (gpt-4-turbo-preview, gpt-3.5-turbo, etc.)
        """
        openai.api_key = api_key
        self.model = model

        # Define available robot actions
        self.available_actions = [
            "navigate(location: str) - Move robot to specified location",
            "detect_objects(filter: str = 'all') - Detect objects in scene",
            "grasp(object: str) - Pick up specified object",
            "place(location: str) - Put down held object at location",
            "open(container: str) - Open container or door",
            "close(container: str) - Close container or door",
            "wait(seconds: float) - Wait for specified time",
        ]

        # Known locations and objects (would be loaded from knowledge base)
        self.known_locations = ["kitchen", "living_room", "bedroom", "garage"]
        self.known_objects = ["cup", "plate", "bottle", "remote", "phone"]

    def plan(self, user_command: str, context: Optional[Dict] = None) -> List[Dict]:
        """
        Generate action plan from natural language command

        Args:
            user_command: Natural language command from user
            context: Optional context (current location, visible objects, etc.)

        Returns:
            List of action steps [{"action": "navigate", "params": {"location": "kitchen"}}, ...]
        """

        prompt = self._create_prompt(user_command, context)

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a robot task planner. Generate concise, executable plans."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,  # Low temperature for consistent planning
                max_tokens=500
            )

            # Parse JSON response
            plan_text = response.choices[0].message.content
            plan = json.loads(plan_text)

            return plan["steps"]

        except Exception as e:
            print(f"Error generating plan: {e}")
            return []

    def _create_prompt(self, command: str, context: Optional[Dict] = None) -> str:
        """Create structured prompt for LLM"""

        context_str = ""
        if context:
            context_str = f"""
Current context:
- Robot location: {context.get('location', 'unknown')}
- Visible objects: {', '.join(context.get('objects', []))}
- Holding: {context.get('holding', 'nothing')}
"""

        prompt = f"""
Task: Generate a robot action plan for this command:
"{command}"

Available actions:
{chr(10).join('- ' + action for action in self.available_actions)}

Known locations: {', '.join(self.known_locations)}
Known objects: {', '.join(self.known_objects)}

{context_str}

Requirements:
1. Output valid JSON with format: {{"steps": [{{"action": "...", "params": {{...}}}}]}}
2. Use only available actions listed above
3. Be concise - use minimum steps necessary
4. If task is impossible, return {{"steps": [], "error": "reason"}}

Example:
Command: "Bring me a cup from the kitchen"
Output: {{"steps": [
    {{"action": "navigate", "params": {{"location": "kitchen"}}}},
    {{"action": "detect_objects", "params": {{"filter": "cup"}}}},
    {{"action": "grasp", "params": {{"object": "cup"}}}},
    {{"action": "navigate", "params": {{"location": "living_room"}}}},
    {{"action": "place", "params": {{"location": "table"}}}}
]}}
"""

        return prompt

    def validate_plan(self, plan: List[Dict]) -> bool:
        """
        Validate that plan is safe and executable

        Args:
            plan: List of action steps

        Returns:
            True if plan is valid
        """

        for step in plan:
            action = step.get("action", "")

            # Check action exists
            if not any(action in a for a in self.available_actions):
                print(f"Invalid action: {action}")
                return False

            # Check required parameters
            params = step.get("params", {})
            if action == "navigate" and "location" not in params:
                print("navigate action requires 'location' parameter")
                return False

            if action == "grasp" and "object" not in params:
                print("grasp action requires 'object' parameter")
                return False

        return True


# Example usage
if __name__ == "__main__":
    planner = LLMTaskPlanner(api_key="your_openai_key_here")

    # Test command
    command = "Go to the kitchen and bring me a cup"
    context = {
        "location": "living_room",
        "objects": ["remote", "book"],
        "holding": "nothing"
    }

    print(f"Command: {command}")
    print("Generating plan...")

    plan = planner.plan(command, context)

    if planner.validate_plan(plan):
        print("Plan generated successfully:")
        for i, step in enumerate(plan, 1):
            print(f"  Step {i}: {step['action']}({step['params']})")
    else:
        print("Plan validation failed")
