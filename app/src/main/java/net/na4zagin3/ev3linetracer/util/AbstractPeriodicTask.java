package net.na4zagin3.ev3linetracer.util;

import android.os.Handler;

import java.util.Timer;
import java.util.TimerTask;

/**
 * 周期的に何か処理を走らせるためのクラス。
 * 周期カウントはこのクラスのインスタンスで作るスレッドで行われる。
 * invokersMethod()の中の処理はこのインスタンスを作成したスレッドで行われる。
 * 継承して使ってください。
 *
 * originally written by http://magelixir.wordpress.com/2011/05/18/androidperiodictask/
 */
public abstract class AbstractPeriodicTask {

    private long period;
    private boolean isDaemon;
    private boolean isCancelled = true;
    private Timer timer;
    private TimerTask timerTask;
    private Handler handler;

    /**
     * periodミリ秒の周期で動かす
     * @param period 周期ミリ秒
     * @param isDaemon 定期処理を行うスレッドをデーモンスレッドで作成するかどうか（false=ユーザースレッド）
     */
    public AbstractPeriodicTask(long period, boolean isDaemon) {
        handler = new Handler();
        this.period = period;
        this.isDaemon = isDaemon;
    }

    /**
     * periodミリ秒の周期で動かす。タイマースレッドはユーザースレッドで作成される
     * @param period 周期ミリ秒
     */
    public AbstractPeriodicTask(long period){
        this(period,false);
    }

    /**
     * 周期タスクの実行を開始する
     */
    public void execute(){

        if(!isCancelled){
            //isCancelledがfalse（=実行中）ならばこのメソッドは実行しない
            return;
        }

        //timerをキャンセルした場合、timer,timerTaskは破棄されるので都度作り直す
        timerTask = new TimerTask(){
            @Override
            public void run() {
                preInvokersMethod();
                handler.post(new Runnable(){
                    @Override
                    public void run() {
                        invokersMethod();
                    }
                });
                postInvokersMethod();
            }
        };
        timer = new Timer(isDaemon);
        timer.scheduleAtFixedRate(timerTask, period, period);

    }

    /**
     * 周期タスクの実行をキャンセルする
     */
    public void cancel(){
        if(timer==null || timerTask==null){
            return;
        }
        timer.cancel();
        timer = null;
        isCancelled = true;
    }

    /**
     * 本インスタンスを作成したスレッド(例えばUIスレッド）で処理させるメソッド
     */
    abstract protected void invokersMethod();

    /**
     * タイマースレッドで処理させるメソッドでinvokersMethodの直前に呼ばれる。
     */
    protected void preInvokersMethod(){
    }

    /**
     * タイマースレッドで処理させるメソッドでinvokersMethodの直後に呼ばれる。
     */
    protected void postInvokersMethod(){
    }

}
