import os
import glob

# merging files
list_out = glob.glob("out_*.txt")
list_out.sort()
outs_str = " ".join(list_out)

merge_cmd = f"cat {outs_str} > out.txt"
os.system(merge_cmd)
print(f"merged: `{merge_cmd}`")

# summarize the report
current_path = os.getcwd()

report_py = os.path.join(current_path, "report_test.py")
report_file = os.path.join(current_path, "out.txt")

report_cmd = f"python3 {report_py} --out_path {report_file}"
os.system(report_cmd)
print(f"report: `{report_cmd}`\n\n")

# plot the report
plot_py = os.path.join(current_path, "plot_data.py")
plot_cmd = f"python3 {plot_py} --out_path {report_file}"
os.system(plot_cmd)
print(f"plot: `{plot_cmd}`")
