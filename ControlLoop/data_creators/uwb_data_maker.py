import numpy as np                                                                                                                 
import pandas as pd                                                                                                                
import os                                                                                                                          
                                                                                                                                    
directory_path = '../'  # Makeshift directory                                                                                      
                                                                                                                                    
# Define function to include all distances                                                                                         
                                                                                                                                    
def generate_combined_uwb_data(filenames, start_distances, end_distances, num_points=10001, noise_std=0.5):                        
    dt = 0.001  # time step (1 ms)                                                                                                 
    time = np.round(np.linspace(0, num_points * dt, num_points), 4)                                                                
                                                                                                                                    
    all_distances = {}                                                                                                             
    for i, (start, end) in enumerate(zip(start_distances, end_distances), 1):                                                      
        distances = np.linspace(start, end, num_points)                                                                            
        noise = np.random.normal(0, noise_std, num_points)                                                                         
        noisy_distances = distances + noise                                                                                        
        all_distances[f'Distance{i}'] = noisy_distances                                                                            
                                                                                                                                    
    # Combine into single DataFrame                                                                                                
    uwb_data_combined = pd.DataFrame({'Time': time, **all_distances})                                                              
                                                                                                                                    
    # Save the combined data into one CSV                                                                                          
    uwb_data_combined.to_csv(os.path.join(directory_path, 'uwb_combined_distances.csv'), index=False)                              
                                                                                                                                    
# Generate combined CSV for all trajectories                                                                                       
generate_combined_uwb_data(                                                                                                        
    ['uwb_trajectory_20_to_50.csv', 'uwb_trajectory_50_to_30.csv', 'uwb_trajectory_additional.csv'],                               
    start_distances=[20, 50, 30],                                                                                                  
    end_distances=[50, 30, 40]                                                                                                     
)            