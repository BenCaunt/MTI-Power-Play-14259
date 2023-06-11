package org.firstinspires.ftc.teamcode.input;

import java.util.ArrayList;

public class AsyncThreaded {
    public static boolean stopped = false;

    private Thread thread;
    private ArrayList<Dependency> dependencies;
    private ArrayList<AsyncThreaded> dependents;

    public AsyncThreaded(Runnable runnable) {
        AsyncThreaded.stopped = false;
        this.dependencies = new ArrayList<>();
        this.dependents = new ArrayList<>();
        this.thread = new Thread(() -> {
            if (AsyncThreaded.stopped) return;
            runnable.run();
            for (AsyncThreaded dependent : this.dependents)
                dependent.update(this);
        });
    }

    public static class Dependency {
        public AsyncThreaded asyncThreaded;
        public boolean fulfilled;

        public Dependency(AsyncThreaded asyncThreaded) {
            this.asyncThreaded = asyncThreaded;
            this.fulfilled = false;
        }
    }

    private void addDependency(AsyncThreaded that) {
        this.dependencies.add(new Dependency(that));
    }

    private void addDependent(AsyncThreaded that) {
        this.dependents.add(that);
    }

    public AsyncThreaded then(AsyncThreaded that) {
        this.addDependent(that);
        that.addDependency(this);
        return that;
    }

    public AsyncThreaded then(Runnable that) {
        AsyncThreaded thatAsync = new AsyncThreaded(that);
        this.addDependent(thatAsync);
        thatAsync.addDependency(this);
        return thatAsync;
    }

    public void run() {
        this.thread.start();
    }

    private void update(AsyncThreaded asyncThreaded) {
        boolean fulfilled = true;
        for (Dependency dependency : this.dependencies) {
            if (asyncThreaded == dependency.asyncThreaded) dependency.fulfilled = true;
            fulfilled = fulfilled && dependency.fulfilled;
        }
        if (!fulfilled) return;
        for (Dependency dependency : this.dependencies) dependency.fulfilled = false;
        this.run();
    }
}
