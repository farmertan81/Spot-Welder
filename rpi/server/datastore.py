import json
import os
from datetime import datetime

class Store:
    def __init__(self, data_dir="weld_data"):
        self.data_dir = data_dir
        os.makedirs(data_dir, exist_ok=True)
        self.welds = []
        self.events = []
    
    def add_weld(self, weld_event):
        """Store a weld event"""
        self.welds.append(weld_event)
        
        # Save to file
        weld_id = weld_event.get("id", 0)
        filename = os.path.join(self.data_dir, f"weld_{weld_id:04d}.json")
        with open(filename, 'w') as f:
            json.dump(weld_event, f, indent=2)
    
    def get_welds(self, limit=50):
        """Get recent welds"""
        return self.welds[-limit:]
    
    def log_event(self, event_type, data):
        """Log an event"""
        event = {
            "type": event_type,
            "timestamp": datetime.now().isoformat(),
            "data": data
        }
        self.events.append(event)
