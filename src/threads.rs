use alloc::boxed::Box;
use core::arch::asm;
use alloc::collections;

/// What states can a thread be in?
/// Most of their life, it's either running or blocked, but there
/// are a some states before and after those:
#[derive(PartialEq, Debug)]
pub enum ThreadStatus {
    BrandNew, // haven't called entry function yet
    Running,  // currently executing (or able to)
    Blocked,  // runnable but waiting for something
    Ended,    // exited entry function
    Zombie,   // undead?
}

/// Our type for a thread entrypoint. Takes a T, returns a T
pub type ThreadEntry<T> = fn(T) -> T;

/// Heap descriptor for a thread. During context switches, the register state
/// gets pushed on to the stack
#[derive(Debug)]
pub struct ThreadState<T> {
    pub saved_sp: usize,
    pub status: ThreadStatus,
    pub function: ThreadEntry<T>,
    pub arg: T,
    pub id: usize,
}

pub struct GlobalState {
    pub run_q: Box<collections::VecDeque<ThreadState<i32>>>,
    // other OS global state goes here
    // "Big mutex?"
    // Device tree
    // ...
}

impl ThreadState<i32> {
    pub fn new(stack: usize, func: ThreadEntry<i32>, arg: i32, id: usize) -> ThreadState<i32> {
        ThreadState {
            status: ThreadStatus::BrandNew,
            saved_sp: stack,
            function: func,
            arg: arg,
            id: id,
        }
    }
}

/// Get a reference to the current thread state.
///
/// # Safety
/// The caller must guarantee that the value pointed to by the riscv $tp register follows rust
/// memory rules, i.e.
/// 1. is not mutably borrowed for the lifetime of the callee
/// 2. lives as long as the callee asks it to
/// 3. is non-null, non-dangled, properly aligned
/// 4. points to a valid `ThreadState`
/// Written during Spring Break 2023 by Riley Shahar
macro_rules! current_thread {
    () => {
        {
            let tp: *mut ThreadState<T>;
            core::arch::asm!("mv {}, tp", out(reg) tp, options(nomem, nostack));
            &mut *tp
        }
    }
}

macro_rules! set_current_thread {
    ($ct:ident) => {
        {
            let ct_ptr: * mut ThreadState<i32> = core::ptr::addr_of_mut!($ct);
            let ct_usize: usize = ct_ptr as usize; // cast through raw pointer first
            core::arch::asm!("mv tp, {}", in(reg) ct_usize, options(nomem, nostack));
        }
    }
}

macro_rules! global_state {
    () => {
        {
            let tp: *mut GlobalState;
            core::arch::asm!("mv {}, gp", out(reg) tp, options(nomem, nostack));
            &mut *tp
        }
    }
}
macro_rules! set_global_state {
    ($gt:ident) => {
        {
            let gs_ptr: * mut GlobalState = core::ptr::addr_of_mut!($gt);
            let gs_usize: usize = gs_ptr as usize; // cast through raw pointer first
            core::arch::asm!("mv gp, {}", in(reg) gs_usize, options(nomem, nostack));
        }
    }
}

/// `do_yield()` gives the scheduler a chance to run someone else.
/// eventually this will acquire the lock on the run queue, find the next runnable thread
/// (if there is one) and switch to it. If there isn't one, it just returns.
pub fn do_yield() {
    log!(Debug, "in yield!");
    let gs_ptr;
    unsafe {
        gs_ptr = global_state!();
        thread_switch(&mut gs_ptr.run_q);
    }
}

// fn example_usage() {
//     // SAFETY: probably unsafe tbh
//     let _: &ThreadState = unsafe { current_thread!() };
// }

/// Switch execution from the current thread to `to_thread`
/// (on RISC-V, current thread struct is pointed to by the `tp` register)
/// (on x86, ... you need to play some tricks - see how Linux does it:
///  https://www.akkadia.org/drepper/tls.pdf)
/// To switch, we push the _callee-saved_ registers to the stack,
/// then switch stacks, then restore the callee_saved registers,
/// then return from this function, which actually returns into
/// the context of `to_thread`. This basically tricks the procedure
/// call mechanism into doing the thread switch for us.
/// This is not so different from the famous v6 routine, whose comment
/// is "you are not expected to understand this":
/// https://en.wikipedia.org/wiki/A_Commentary_on_the_UNIX_Operating_System
/// One difference is, you _are_ expected to understand this, at least a bit
//pub extern "C" fn thread_switch<T: Copy>(to_thread: &mut ThreadState<T>) {
pub extern "C" fn thread_switch<T: Copy>(run_q: &mut Box<collections::VecDeque<ThreadState<T>>>) {
    // On x86, callee saved registers are:
    //   ebp, ebx, edi, esi (and of course esp)
    // On RISC-V, they are:
    //   sp, gp, tp, s0-11
    // we are one "callee" who does *not* preserve tp or sp
    //   - it's our job, in fact

    // let to_thread = run_q.get(0).unwrap(); // todo: never fail
    let to_thread = run_q.get(0).unwrap(); // todo: never fail
    #[cfg(target_arch = "riscv64")]
    println!("riscv switching to {0}", to_thread.id);
    unsafe {
        asm!(
            "mv t3, {to_thread}", // save to_thread in t3 (a caller-saved temp register)
            "addi sp, sp, -0x68",
            "sw s0,  0x00(sp)",
            "sw s1,  0x08(sp)",
            "sw s2,  0x10(sp)",
            "sw s3,  0x18(sp)",
            "sw s4,  0x20(sp)",
            "sw s5,  0x28(sp)",
            "sw s6,  0x30(sp)",
            "sw s7,  0x38(sp)",
            "sw s8,  0x40(sp)",
            "sw s9,  0x48(sp)",
            "sw s10, 0x50(sp)",
            "sw s11, 0x58(sp)",
            "sw tp,  0x60(sp)",
            "mv sp, {dest_sp}", // switching stacks! you're expected to understand this ;)
            "mv tp, t3",        // this is doing the work of set_current_thread
            to_thread = in(reg) to_thread,
            dest_sp = in(reg) (to_thread.saved_sp),
            // "addi sp, sp, 0x68"
        );
        // if we're switching to a brand-new thread, call its function instead.
        // the `tp` register points to the dest-thread's ThreadState, so let's use that:
        let ct = current_thread!();
        match ct.status {
            ThreadStatus::BrandNew => {
                ct.status = ThreadStatus::Running;
                (ct.function)(ct.arg);
                panic!("thread returned ... we should handle that nicely");
            }
            ThreadStatus::Running => {
                // we have a thread to switch to!
                // fall through to thread switchk
            }
            _ => {
                panic!("invalid thread status in thread_switch {:?}", ct.status);
            }
        }

        println!("about to switch here...");
        asm!(
            "lw s0,  0x00(sp)",
            "lw s1,  0x08(sp)",
            "lw s2,  0x10(sp)",
            "lw s3,  0x18(sp)",
            "lw s4,  0x20(sp)",
            "lw s5,  0x28(sp)",
            "lw s6,  0x30(sp)",
            "lw s7,  0x38(sp)",
            "lw s8,  0x40(sp)",
            "lw s9,  0x48(sp)",
            "lw s10, 0x50(sp)",
            "lw s11, 0x58(sp)",
            "lw tp,  0x60(sp)",
            "addi sp, sp, 0x68",
        );
        // println!("state restored, returning to dest thread.");
    }
}

fn dummy_entry(_my_arg: i32) -> i32 {
    println!("hello from dummy?");
    loop {}
}

// // Ideally: move this _start to lib.rs, and have it
// // call whichever main is defined in the `bin/foo` we're building
// #[no_mangle] // don't mangle the name of this function
// pub extern "C" fn _start() -> ! {
//     let mut main_thread_stack:[u8;4096] = [0; 4096];
//     main_thread_stack[0] = 0xfe;
//     let main_thread_sp = (main_thread_stack.as_ptr() as usize) + 4000;
//     let mut main_thread_state = ThreadState::new(main_thread_sp, dummy_entry, 0, 42);
//     main_thread_state.status = ThreadStatus::Running;
//     main();
//     println!("main() is done");
//     unsafe { libc::exit(0); }
// }

fn thread_test(my_arg: i32) -> i32 {
    println!("hello - my arg was {}", my_arg);
    for _ in 1..10 {
        do_yield();
    }
    my_arg + 1
}

// TODO: allocate a run_q, set a pointer to it in gp, access it in do_yield
pub fn test_threads() {
    let mut run_q: Box<collections::VecDeque<ThreadState<i32>>> = Box::default();

    log!(Debug, "hey from test_threads");
    let to_thread_stack = crate::vm::palloc_plural(2).unwrap();
    let mut to_thread = ThreadState::new(
        to_thread_stack as usize + crate::param::PAGE_SIZE * 2 - 16,
        thread_test,
        42,
        42,
    );
    let mut two_thread = ThreadState::new(
        to_thread_stack as usize + crate::param::PAGE_SIZE * 2 - 16,
        thread_test,
        43,
        43,
    );
    // shouldn't need this, but just to be super-clean
    let from_thread_stack = crate::vm::palloc_plural(2).unwrap();
    let mut from_thread = ThreadState::new(
        from_thread_stack as usize + crate::param::PAGE_SIZE * 2 - 16,
        dummy_entry,
        99,
        99,
    );
    from_thread.status = ThreadStatus::Running;
    run_q.push_back(two_thread);
    run_q.push_back(to_thread);
    // run_q.push_back(from_thread);

    unsafe {
        let mut gs = GlobalState {
            run_q : run_q,
        };
        set_global_state!(gs);
        set_current_thread!(from_thread);
    }
    // thread_switch(&mut run_q);
    do_yield();

    log!(Debug, "OOPS, I'm back in test_threads");
}

// status:
// crashes when the test thread returns.
// todo: 
// - a per-thread run_q
// - a global_run_q
// - yield() switches to run_q, if empty, polls a while, then steals work
//   - this will let us test switching _to_ a thread
// - maybe polling should be a default thread which spin-yields forever?
