import pybullet as p
import trimesh
import os

def main():
    # 1. Use 'os' to dynamically generate absolute paths
    # This works perfectly as long as you run the script from the 'Pybullet_project' root
    input_stl = os.path.abspath("robots/meshes/chassis_link.STL")
    temp_obj = os.path.abspath("robots/meshes/chassis_link_temp.obj")
    final_vhacd_obj = os.path.abspath("robots/meshes/chassis_link_vhacd.obj")
    log_file = os.path.abspath("vhacd_log.txt")

    # 2. Convert the STL to OBJ using trimesh
    print("--- STEP 1: Converting STL to OBJ ---")
    print(f"Looking for: {input_stl}")
    
    # Safety check to ensure the file is exactly where we think it is
    if not os.path.exists(input_stl):
        print(f"ERROR: Cannot find {input_stl}.")
        print("Please ensure your terminal is in the 'Pybullet_project' folder.")
        return

    mesh = trimesh.load(input_stl)
    mesh.export(temp_obj)
    print(f"Success! Temporary OBJ created at: {temp_obj}")

    # 3. Run PyBullet's V-HACD on the newly created OBJ
    print("\n--- STEP 2: Running V-HACD on the new OBJ ---")
    print("Processing... (This might take a moment)")
    
    p.connect(p.DIRECT)
    # p.vhacd(
    #     temp_obj,
    #     final_vhacd_obj,
    #     log_file,
    #     concavity=0.0005,  # Forces the algorithm to respect tiny holes
    #     resolution=500000, # Much higher detail
    #     depth=32,          # Slices the mesh more times
    #     maxNumVerticesPerCH=128
    # )
    p.vhacd(
        temp_obj,
        final_vhacd_obj,
        log_file,
        resolution=5000000,     # Increased from 100k to 5 Million voxels to map tiny gaps.
        concavity=0.00001,      # Forces the math to trace the exact lip of the hole instead of smoothing over it.
        depth=32,               # Maximum slicing depth so it can build complex internal walls.
        minVolumePerCH=0.00001, # Allows the engine to generate tiny convex pieces for tight corners.
        maxNumVerticesPerCH=128 # Gives each piece more detail.
    )
    p.disconnect()

    # 4. Verify the final file exists
    if os.path.exists(final_vhacd_obj):
        print("\n--- DONE ---")
        print(f"Your final collision mesh is ready here: {final_vhacd_obj}")
        
        # Optional: Uncomment the next line if you want Python to automatically delete the temp_obj
        # os.remove(temp_obj) 
    else:
        print("\n--- ERROR ---")
        print("V-HACD failed to generate the final mesh.")

if __name__ == "__main__":
    main()