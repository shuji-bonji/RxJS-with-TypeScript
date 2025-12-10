---
description: Questa guida spiega come configurare un template di sviluppo per l'apprendimento costruito con Vite, TypeScript e RxJS. Include hot reloading, ideale per sperimentare con il codice e manipolare il DOM nel browser, oltre allo sviluppo test-driven usando Vitest.
---

# Configurazione dell'Ambiente di Apprendimento Pratico

Questa pagina spiega come utilizzare il template di sviluppo [`RxJS-with-TypeScript-Starter-Kit`](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit), che permette di testare istantaneamente il codice RxJS localmente invece che nel browser.

## Caratteristiche

- Setup semplice: Vite + TypeScript + RxJS
- Supporto hot reload (esegui `npm run dev` per testare immediatamente)
- Ambiente di sviluppo locale che supporta manipolazione DOM e testing
- Test-Driven Development (TDD) abilitato con Vitest

## Utilizzo

Clona e configura usando i seguenti comandi:

```bash
git clone https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit.git
cd rxjs-with-typescript-starter-kit
npm install
npm run dev
```

Il browser si avvierà automaticamente ed eseguirà il codice scritto in `src/main.ts`.

## Esempio di Utilizzo

Riscrivi il file `src/main.ts` esistente come segue.

```ts
// src/main.ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `Conteggio: ${val}`;
  output.appendChild(p);
});
```

### Accesso localhost
L'URL sarà visualizzato come `http://localhost:5174/`. Accedi a questo URL per vedere i risultati.
Per controllare i risultati di `console.log()`, usa la console nei tuoi strumenti per sviluppatori.

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

## Usi Consigliati

- Sperimentare con Observable / Subject / Operatori
- Imparare UI reattive combinate con DOM
- Praticare implementazione di marble test (`vitest` + `TestScheduler`)
- Ambiente base per memorizzare i tuoi snippet di codice

## Link

🔗 Template qui → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)
Vedi `README.md` per dettagli.
