/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author gokul
 */
public class Solution {
    // DO NOT MODIFY THE ARGUMENTS WITH "final" PREFIX. IT IS READ ONLY
    public int[] spiralOrder(final int[][] A) {
        int left = 0;
        int top = 0;
        int right = A.length;
        if(right == 0){
            return A;
        }
        int bottom = A[0].length;
        System.out.print(bottom);
        int[] ans = new int[right*bottom];
        int i = 0; int j = 0;
        int dir = 0;
        int k = 0;
        /**
        if 
        while(){
            if(dir==0){
                ans[k] = A[i][j];
                j += 1;
                if(j == right){
                    dir = 1;
                }
            if(dir == 1){
                ans[k] = A[][]
            }
            k++;
            }
        }
        */
        return ans;
    }
}