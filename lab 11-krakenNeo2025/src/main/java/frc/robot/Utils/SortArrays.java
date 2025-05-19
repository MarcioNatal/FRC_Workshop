// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Add your docs here. */
public class SortArrays 
{
    public static <T extends Comparable<T>> void concurrentSort(
                                        final List<T> key, List<?>... lists){
    // Create a List of indices
    List<Integer> indices = new ArrayList<Integer>();
    for(int i = 0; i < key.size(); i++)
        indices.add(i);

    // Sort the indices list based on the key
    Collections.sort(indices, new Comparator<Integer>(){
        @Override public int compare(Integer i, Integer j) {
            return key.get(i).compareTo(key.get(j));
        }
    });

    // Create a mapping that allows sorting of the List by N swaps.
    // Only swaps can be used since we do not know the type of the lists
    Map<Integer,Integer> swapMap = new HashMap<Integer, Integer>(indices.size());
    List<Integer> swapFrom = new ArrayList<Integer>(indices.size()),
                  swapTo   = new ArrayList<Integer>(indices.size());
    for(int i = 0; i < key.size(); i++){
        int k = indices.get(i);
        while(i != k && swapMap.containsKey(k))
            k = swapMap.get(k);

        swapFrom.add(i);
        swapTo.add(k);
        swapMap.put(i, k);
    }

    // use the swap order to sort each list by swapping elements
    for(List<?> list : lists)
        for(int i = 0; i < list.size(); i++)
            Collections.swap(list, swapFrom.get(i), swapTo.get(i));
    }
}

