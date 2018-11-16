import h5py


def h5_structure(h5_group, show_attrs=True, show_data=False):
    """
    Recursively print the tree structure of an HDF5 file or group
    """
    sep = '| '
    def print_group_structure(h5_group, str_prefix):
        for name, group in h5_group.items():
            if isinstance(group, h5py.Dataset):
                if show_data:
                    print(str_prefix + name + ": " + str(group[...]))
                else:
                    print(str_prefix + str(group))
                if show_attrs:
                    print_attributes(group, str_prefix + sep)
            else:
                print(str_prefix + name)
                print_group_structure(group, str_prefix+sep)
    
    def print_attributes(dset, str_prefix):
        for k, v in dset.attrs.items():
            print(str_prefix, k, ':', v)
    
    print_group_structure(h5_group, '')

h5file = h5py.File('test.h5', 'r')
h5_structure(h5file, show_data=True)


h5file.close()
