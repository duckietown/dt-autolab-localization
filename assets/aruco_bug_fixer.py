FILE_ARR = []
OLD_ARR = []
NEW_ARR = []

FILE_ARR.append('aruco-3.1.12/src/markerdetector_impl.cpp')
OLD_ARR.append('MarkerCanditates[i]=corners[i];')
NEW_ARR.append('    {                                                     '
               '        MarkerCanditates[i].resize(corners[i].size());    '
               '        for (int j = 0; j < corners[i].size(); j++) {     '
               '            MarkerCanditates[i][j] = corners[i][j];       '
               '        }                                                 '
               '    }                                                     ')

FILE_ARR.append('aruco-3.1.12/src/dcf/dcfmarkertracker.cpp')
OLD_ARR.append('std::cout << "ArUco Detected: "<< detectedMarkers.size() <<" new markers"<< std::endl;')
NEW_ARR.append('')

if __name__ == '__main__':
    """
    Fixes bug with vectors assignment and removes unnecessary printing in ArUco library
    """
    for i in range(len(FILE_ARR)):
        found = False
        with open(FILE_ARR[i], 'r') as f:
            lines = f.readlines()
        with open(FILE_ARR[i], 'w+') as f:
            for line in lines:
                if line.strip() == OLD_ARR[i]:
                    f.write(NEW_ARR[i])
                    found = True
                else:
                    f.write(line)
        if not found:
            raise ValueError(f"No lines replaced in {FILE_ARR[i]}")
