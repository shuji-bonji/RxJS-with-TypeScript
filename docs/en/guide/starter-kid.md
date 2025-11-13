---
Description: This guide explains how to set up a learning development template built with Vite, TypeScript, and RxJS. It features hot reloading, making it ideal for experimenting with code and manipulating the DOM in the browser, as well as test-driven development using Vitest.
---

# Setting Up the Hands-On Learning Environment

This page explains how to use the [`RxJS-with-TypeScript-Starter-Kit`](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit) development template, which lets you instantly test RxJS code locally instead of in a browser.

## Features

- Simple setup: Vite + TypeScript + RxJS
- Hot reload support (run `npm run dev` to test immediately)
- Local development environment supporting DOM manipulation and testing
- Test-Driven Development (TDD) enabled with Vitest

## Usage

Clone and set up using the following commands:

```bash
git clone https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit.git
cd rxjs-with-typescript-starter-kit
npm install
npm run dev
```

Your browser will launch automatically and execute the code written in `src/main.ts`.

## Usage Example

Rewrite the existing `src/main.ts` as follows.

```ts
// src/main.ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `Count: ${val}`;
  output.appendChild(p);
});
```

### Access localhost
The URL will be displayed as `http://localhost:5174/`. Access this URL to see the results.  
To check the results of `console.log()`, use the console in your developer tools.

```sh
% npm run dev

> rxjs-with-typescript-starter-kit@0.0.0 dev
> vite

Port 5173 is in use, trying another one...

  VITE v6.3.1  ready in 107 ms

  ➜  Local:   http://localhost:5174/
  ➜  Network: use --host to expose
  ➜  press h + enter to show help
```

## Recommended Uses

- Experimenting with Observables / Subjects / Operators
- Learning reactive UI combined with DOM
- Practicing marble test implementation (`vitest` + `TestScheduler`)
- Base environment for storing your own code snippets

## Links

🔗 Template here → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)  
See `README.md` for details.