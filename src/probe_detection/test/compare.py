import pandas as pd

def compare_vital_results(file1_path, file2_path, output_path):
    # Read the CSV files
    df1 = pd.read_csv(file1_path)
    df2 = pd.read_csv(file2_path)
    
    # Merge DataFrames on image_id and probe_id
    merged_df = df1.merge(df2, on=['image_id', 'probe_id'], suffixes=('_csv1', '_csv2'))
    
    # Calculate error difference
    merged_df['error_diff'] = merged_df['error_csv1'] - merged_df['error_csv2']
    
    # Select relevant columns for output
    result_df = merged_df[['image_id', 'probe_id', 'error_csv1', 'error_csv2', 'error_diff']]
    
    # Save results to CSV
    result_df.to_csv(output_path, index=False)
    
    # Compute summary statistics
    summary_stats = {
        'mean_error_diff': merged_df['error_diff'].mean(),
        'max_error_diff': merged_df['error_diff'].max(),
        'min_error_diff': merged_df['error_diff'].min(),
        'std_error_diff': merged_df['error_diff'].std()
    }
    
    # Print summary
    print("Summary of Error Differences:")
    for stat, value in summary_stats.items():
        print(f"{stat.replace('_', ' ').title()}: {value:.6f}")
    
    return result_df

if __name__ == "__main__":
    file1_path = "vital_results.csv"
    file2_path = "vital_results2.csv"
    output_path = "error_comparison.csv"
    
    try:
        compare_vital_results(file1_path, file2_path, output_path)
        print(f"\nComparison results saved to {output_path}")
    except FileNotFoundError:
        print("Error: One or both input files not found.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")