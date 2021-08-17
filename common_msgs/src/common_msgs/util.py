def convert_covs(*cov_ls):
  if None in cov_ls: return None
  converted_cov = [0]*((len(cov_ls)*3)**2)
  for i, cov in enumerate(cov_ls):
    for j in range(3):
      converted_cov[(i*3+j)*7] = cov[j]
  return converted_cov