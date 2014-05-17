package ru.getsk.common.helper;

import java.util.Random;

public class RandomHelper {
    private static Random mRandom = new Random();

    /**
     *
     * @param min Lower bound of random interval
     * @param max Upper bound of random interval
     * @return Random integer number from min to max interval
     */
    public static int randInt(int min, int max) {
        return mRandom.nextInt((max - min) + 1) + min;
    }
}
