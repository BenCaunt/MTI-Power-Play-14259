//package com.example.usaco;
//
///*
//5 6 1 5
//1 2 1
//2 3 3
//2 5 100
//3 4 3
//3 5 5
//4 5 3
//4 1 2 3 5
// */
//
//import java.util.ArrayList;
//import java.util.Comparator;
//import java.util.Scanner;
//
//public class ShortestPaths {
//    static class Edge {
//        public int a, b, w;
//
//        public Edge(int a, int b, int w) {
//            this.a = a;
//            this.b = b;
//            this.w = w;
//        }
//    }
//
//    static ArrayList<Edge> edges;
//    static ArrayList<Integer> luckyPath;
//
//    public static void main(String[] args) {
//        Scanner scanner = new Scanner(System.in);
//        int n = scanner.nextInt(), m = scanner.nextInt(), a = scanner.nextInt() - 1, b = scanner.nextInt() - 1;
//        for (int i = 0; i < m; i++) {
//            int u = scanner.nextInt() - 1, v = scanner.nextInt() - 1, w = scanner.nextInt();
//            edges.add(new Edge(u, v, w));
//        }
//        int k = scanner.nextInt();
//        for (int i = 0; i < k; i++) {
//            int v = scanner.nextInt() - 1;
//            luckyPath.add(v);
//        }
//        edges.sort(new Comparator<Edge>() {
//            @Override
//            public int compare(Edge a, Edge b) {
//                return a.w - b.w;
//            }
//        })
//    }
//}