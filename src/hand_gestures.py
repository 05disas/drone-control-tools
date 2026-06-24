class HandGestures:

    #Notes
    """
    Figure out a way to detect if the thumb is up or down.
    The current approach I'm thinking is check the distance from the tip of the thumb to the 
    pip of the middles finger. The reason is there are two ways the thumb can be down. One is the 
    thumb can be under the the rest of the fingers and the other is the thumb can be sitting left
    to the index finger. In both cases the distance from the tip of the thumb to the pip of the 
    middle finger will be less when the thumb is down. When the hand is further away from the camera the distance
    will be different so a scaling factor will have to be used. This scaling factor can be calculated by measuring the 
    distance from the knuckle of the index finger to the wrist since this distance in unlikely 
    to change.
    We can verify if the hand is vertical by checking if the thumb_cmc is located higher than the ring mcp
    """




    def __init__(self):
        #All fingures are bent
        self.FIST = 0
        #All fingures are straight
        self.PALM = 5
        #Index finger is up (Thumb can be up or down other fingers are bent)
        self.INDEX_UP = 1
        #Index and middle fingers are up (Thumb can be up or down other fingers are bent)
        self.INDEX_MIDDLE_UP = 2
        #Thumb is up (Other fingers can be up or down)
        #Note: Thumb has to point upwards
        self.THUMB_UP = 6
        #Thumb is down (Other fingers can be up or down)
        #Note: Thumb has to point downwards
        self.THUMB_DOWN = 9

        self._thumb_threshold = 0.1 
        self._thumb_threshold_scaling_factor = 1.0  # Adjust this scaling factor as needed

    def get_gesture(self, hand_landmarks):
        #This function will analyze the hand landmarks and return the corresponding gesture

        if hand_landmarks == None:
            return -1
        
        landmarks = hand_landmarks.landmark

        #IDs for finger tips (Thumb, Index, Middle, Ring, Pinky)
        tips = [4, 8, 12, 16, 20]
        #IDs for finger pip joints (Thumb, Index, Middle, Ring, Pinky)
        pips = [3, 6, 10, 14, 18]

        #Thumbs up and down has to be calculated before FIST and PALM (and all other gestures) because when the 
        #thumb is horizontal the logic for FIST and PALM will not work.
        #By checking for the thumb frist we can use the same logic for FIST and PALM since if the thumbs
        #up is detected then the function will return and not check for FIST or PALM.

        #Count how many fingers are up
        #Thumb will not be used for this count
        #(Can we use a different logic when the hand is vertical?)
        fingers_up = 0
        for i in range(1, 5):
            if landmarks[tips[i]].y < landmarks[pips[i]].y:
                fingers_up += 1



        #Check if thumb is up
        #Calculate distance from tip of thumb to pip of middle finger
        thumb_tip = landmarks[tips[0]]
        middle_pip = landmarks[pips[2]]
        thumb_to_middle_distance = ((thumb_tip.x - middle_pip.x) ** 2 + (thumb_tip.y - middle_pip.y) ** 2) ** 0.5
        #Calculate thumb threshold based on distance from knuckle of index finger to wrist
        index_knuckle = landmarks[5]
        wrist = landmarks[0]
        self._thumb_threshold = self._thumb_threshold_scaling_factor * ((index_knuckle.x - wrist.x) ** 2 + (index_knuckle.y - wrist.y) ** 2) ** 0.5

        # Check if hand is vertical. 
        # A robust way is to check if the Index knuckle (5) and Pinky knuckle (17) are stacked 
        # vertically rather than sitting horizontally side-by-side.
        index_mcp = landmarks[5]
        pinky_mcp = landmarks[17]
        is_hand_vertical = abs(index_mcp.y - pinky_mcp.y) > abs(index_mcp.x - pinky_mcp.x)

        # 1. Is the thumb extended away from the palm?
        is_thumb_extended = thumb_to_middle_distance > self._thumb_threshold

        # 2. Is it pointing strictly up or strictly down?
        # We verify direction by checking the sequence of the thumb joints
        thumb_ip = landmarks[3]
        thumb_mcp = landmarks[2]
        
        is_pointing_up = (thumb_tip.y < thumb_ip.y) and (thumb_ip.y < thumb_mcp.y)
        is_pointing_down = (thumb_tip.y > thumb_ip.y) and (thumb_ip.y > thumb_mcp.y)

        # 3. Evaluate and return immediately
        if is_thumb_extended and is_hand_vertical:
            if is_pointing_up:
                return self.THUMB_UP
            elif is_pointing_down:
                return self.THUMB_DOWN
        







        if fingers_up == 0:
            return self.FIST
        
        #Check if index finger is up
        if fingers_up == 1 and landmarks[tips[1]].y < landmarks[pips[1]].y:
            return self.INDEX_UP
        #Check if index and middle fingers are up
        if fingers_up == 2 and landmarks[tips[1]].y < landmarks[pips[1]].y and landmarks[tips[2]].y < landmarks[pips[2]].y:
            return self.INDEX_MIDDLE_UP



        if fingers_up == 4:
            return self.PALM
        


    