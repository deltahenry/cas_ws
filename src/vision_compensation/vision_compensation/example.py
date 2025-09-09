#!/usr/bin/env python3
"""
Example usage of vision compensation functions.
Translated from utility/example.cs
"""

import cv2
import numpy as np
import os
from vision import (compensate_cabinet, compensate_cabinet_test, 
                   compensate_cabinet_test_with_visualization, 
                   compensate_cabinet_with_visualization, BlobEdgePipeline)

def run_example():
    """Example function demonstrating vision compensation usage"""
    
    # Create pipeline configurations using BlobEdgePipeline class
    # Try to load from config file, fall back to defaults
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'vision_parameter.yaml')
    pipe_g = BlobEdgePipeline(config_path if os.path.exists(config_path) else None)
    pipe_c = BlobEdgePipeline(config_path if os.path.exists(config_path) else None)
    
    print(f"Pipeline configuration loaded:")
    print(f"  MinAreaLR: {pipe_g.MinAreaLR}")
    print(f"  LeftRightThreshold: {pipe_g.LeftRightThreshold}")
    print(f"  MinAreaTB: {pipe_g.MinAreaTB}")
    print(f"  TopBottomThreshold: {pipe_g.TopBottomThreshold}")
    
    # Example image paths (adjust these paths based on your setup)
    golden_path = "2D(x=0,z=107).png"
    current_path = "2D(x=0,z=117).png"
    
    print("Running vision compensation example...")
    print(f"Golden image: {golden_path}")
    print(f"Current image: {current_path}")
    
    # Check if images exist
    if not os.path.exists(golden_path):
        print(f"Warning: Golden image not found at {golden_path}")
        print("Using default test images from home directory...")
        golden_path = os.path.expanduser("~/image_sample/2D_golden(x=0,z=112).png")
        current_path = os.path.expanduser("~/image_sample/2D(x=25,z=117).png")
    
    try:
        # Load images using OpenCV
        gold_bgr = cv2.imread(golden_path, cv2.IMREAD_COLOR)
        current_bgr = cv2.imread(current_path, cv2.IMREAD_COLOR)
        
        if gold_bgr is None:
            print(f"Error: Could not load golden image from {golden_path}")
            return
            
        if current_bgr is None:
            print(f"Error: Could not load current image from {current_path}")
            return
        
        # Convert to grayscale (equivalent to Convert<Gray, byte>() in C#)
        gold_gray = cv2.cvtColor(gold_bgr, cv2.COLOR_BGR2GRAY)
        current_gray = cv2.cvtColor(current_bgr, cv2.COLOR_BGR2GRAY)
        
        print("Images loaded successfully")
        print(f"Golden image size: {gold_gray.shape}")
        print(f"Current image size: {current_gray.shape}")
        
        # Perform compensation calculation using the current image directly
        try:
            # Use the new visualization function 
            x, z, vis_golden, vis_current = compensate_cabinet_with_visualization(golden_path, current_bgr)
            
            print(f"Compensation calculated successfully!")
            print(f"ΔX={x:.2f}, ΔZ={z:.2f}")
            
            # Apply calibration coefficients as per specification
            x_calibrated = x * 0.53
            z_calibrated = z * 0.18
            print(f"Calibrated: ΔX={x_calibrated:.2f}, ΔZ={z_calibrated:.2f}")
            
            # Save visualization images
            if vis_golden is not None:
                cv2.imwrite("example_golden_with_corners.png", vis_golden)
                print("Saved: example_golden_with_corners.png")
                
            if vis_current is not None:
                cv2.imwrite("example_current_with_corners.png", vis_current)
                print("Saved: example_current_with_corners.png")
            
            # Display the two output images as required
            if vis_golden is not None and vis_current is not None:
                print("Displaying output images...")
                try:
                    cv2.imshow("Golden Image with Corners", vis_golden)
                    cv2.imshow("Current Image with Corners", vis_current)
                    print("Press any key to continue...")
                    cv2.waitKey(0)  # Wait for key press
                    cv2.destroyAllWindows()
                    print("Images displayed successfully")
                except Exception as e:
                    print(f"Display not available: {e}")
                    print("Images saved to files instead")
            
        except Exception as e:
            print(f"Error in compensation calculation: {str(e)}")
            print("Could not compute average corners.")
            
    except Exception as e:
        print(f"Error processing images: {str(e)}")

def run_test_example():
    """Example using test images from files"""
    
    print("\nRunning test mode example...")
    
    golden_path = os.path.expanduser("~/image_sample/2D_golden(x=0,z=112).png")
    test_path = os.path.expanduser("~/image_sample/2D(x=25,z=117).png")
    
    print(f"Golden image: {golden_path}")
    print(f"Test image: {test_path}")
    
    try:
        x, z, vis_golden, vis_current = compensate_cabinet_test_with_visualization(golden_path, test_path)
        
        print(f"Test compensation calculated successfully!")
        print(f"ΔX={x:.2f}, ΔZ={z:.2f}")
        
        # Apply calibration coefficients as per specification
        x_calibrated = x * 0.53
        z_calibrated = z * 0.18
        print(f"Calibrated: ΔX={x_calibrated:.2f}, ΔZ={z_calibrated:.2f}")
        
        # Save visualization images
        if vis_golden is not None:
            cv2.imwrite("test_golden_with_corners.png", vis_golden)
            print("Saved: test_golden_with_corners.png")
            
        if vis_current is not None:
            cv2.imwrite("test_current_with_corners.png", vis_current)
            print("Saved: test_current_with_corners.png")
        
        # Display the two output images as required
        if vis_golden is not None and vis_current is not None:
            print("Displaying test images...")
            try:
                cv2.imshow("Test Golden Image with Corners", vis_golden)
                cv2.imshow("Test Current Image with Corners", vis_current)
                print("Press any key to continue...")
                cv2.waitKey(0)  # Wait for key press
                cv2.destroyAllWindows()
                print("Test images displayed successfully")
            except Exception as e:
                print(f"Display not available: {e}")
                print("Images saved to files instead")
        
    except Exception as e:
        print(f"Error in test compensation calculation: {str(e)}")

def demo_blob_edge_pipeline():
    """Demonstrate BlobEdgePipeline configuration usage"""
    
    print("\nBlobEdgePipeline Configuration Demo")
    print("=" * 40)
    
    # Create pipeline with default parameters
    default_pipeline = BlobEdgePipeline()
    print("Default pipeline parameters:")
    print(f"  MinAreaLR: {default_pipeline.MinAreaLR}")
    print(f"  LeftRightThreshold: {default_pipeline.LeftRightThreshold}")
    print(f"  MinAreaTB: {default_pipeline.MinAreaTB}")
    print(f"  TopBottomThreshold: {default_pipeline.TopBottomThreshold}")
    print(f"  TrimPercent: {default_pipeline.TrimPercent}")
    print(f"  BandScale: {default_pipeline.BandScale}")
    
    # Try to load from config file
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'vision_parameter.yaml')
    if os.path.exists(config_path):
        config_pipeline = BlobEdgePipeline(config_path)
        print(f"\nLoaded from config file: {config_path}")
        print(f"  MinAreaLR: {config_pipeline.MinAreaLR}")
        print(f"  LeftRightThreshold: {config_pipeline.LeftRightThreshold}")
        print(f"  MinAreaTB: {config_pipeline.MinAreaTB}")
        print(f"  TopBottomThreshold: {config_pipeline.TopBottomThreshold}")
    else:
        print(f"\nConfig file not found: {config_path}")
        print("Using default parameters")

def run_visualization_example():
    """Example with visualization output (matching C# behavior)"""
    
    print("\nRunning visualization example...")
    
    golden_path = os.path.expanduser("~/image_sample/2D_golden(x=0,z=112).png")
    test_path = os.path.expanduser("~/image_sample/2D(x=25,z=117).png")
    
    print(f"Golden image: {golden_path}")
    print(f"Test image: {test_path}")
    
    try:
        # Get compensation values and visualization images
        x, z, vis_golden, vis_current = compensate_cabinet_test_with_visualization(golden_path, test_path)
        
        print(f"Visualization compensation calculated successfully!")
        print(f"ΔX={x:.2f}, ΔZ={z:.2f}")
        print(f"Golden visualization image shape: {vis_golden.shape}")
        print(f"Current visualization image shape: {vis_current.shape}")
        
        # Save visualization images
        cv2.imwrite("output_golden_visualization.png", vis_golden)
        cv2.imwrite("output_current_visualization.png", vis_current)
        print("Visualization images saved:")
        print("  - output_golden_visualization.png (shows golden image with corners and average point)")
        print("  - output_current_visualization.png (shows current image with corners, average point, and arrow to golden)")
        
        # Display images as required (improved behavior)
        try:
            cv2.imshow("Golden Sample with Analysis", vis_golden)
            cv2.imshow("Current Image with Analysis", vis_current)
            print("Press any key to continue...")
            cv2.waitKey(0)  # Wait for key press instead of timeout
            cv2.destroyAllWindows()
            print("Visualization images displayed successfully")
        except Exception as e:
            print(f"Display not available: {e}")
            print("Images saved to files instead")
        
    except Exception as e:
        print(f"Error in visualization compensation calculation: {str(e)}")

if __name__ == "__main__":
    print("Vision Compensation Example")
    print("=" * 40)
    
    # Demonstrate BlobEdgePipeline configuration
    demo_blob_edge_pipeline()
    
    # Run the main example
    run_example()
    
    # Run the test example
    run_test_example()
    
    # Run the visualization example (NEW - matches C# output)
    run_visualization_example()
    
    print("\nExample completed.")