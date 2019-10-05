use structopt::{clap::{Shell}, StructOpt};

use super::{Opt};

#[derive(Debug, StructOpt)]
pub struct Completions {

    /// The shell to generate the script for
    pub shell: Shell,
}

impl Completions { 
    pub fn handle(opt: Completions) {
        Opt::clap().gen_completions_to(
            "bbr",
            opt.shell,
            &mut std::io::stdout());
        std::process::exit(0)
    }
}