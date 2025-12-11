---
Description: Deze gids legt uit hoe u een leersontwikkelingstemplate opzet gebouwd met Vite, TypeScript en RxJS. Het beschikt over hot reloading, waardoor het ideaal is voor het experimenteren met code en het manipuleren van de DOM in de browser, evenals test-gedreven ontwikkeling met Vitest.
---

# Opzetten van de Praktische Leeromgeving

Deze pagina legt uit hoe u de [`RxJS-with-TypeScript-Starter-Kit`](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit) ontwikkelingstemplate gebruikt, waarmee u RxJS-code direct lokaal kunt testen in plaats van in een browser.

## Functies

- Eenvoudige setup: Vite + TypeScript + RxJS
- Hot reload ondersteuning (voer `npm run dev` uit om direct te testen)
- Lokale ontwikkelomgeving met ondersteuning voor DOM-manipulatie en testen
- Test-Driven Development (TDD) mogelijk met Vitest

## Gebruik

Kloon en stel in met de volgende commando's:

```bash
git clone https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit.git
cd rxjs-with-typescript-starter-kit
npm install
npm run dev
```

Uw browser wordt automatisch gestart en voert de code uit die geschreven is in `src/main.ts`.

## Gebruiksvoorbeeld

Herschrijf de bestaande `src/main.ts` als volgt.

```ts
// src/main.ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `Teller: ${val}`;
  output.appendChild(p);
});
```

### Toegang tot localhost
De URL wordt weergegeven als `http://localhost:5174/`. Ga naar deze URL om de resultaten te bekijken.
Om de resultaten van `console.log()` te controleren, gebruik de console in uw ontwikkelaarstools.

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

## Aanbevolen Toepassingen

- Experimenteren met Observables / Subjects / Operators
- Leren van reactieve UI gecombineerd met DOM
- Oefenen met marble test implementatie (`vitest` + `TestScheduler`)
- Basisomgeving voor het opslaan van uw eigen codefragmenten

## Links

🔗 Template hier → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)
Zie `README.md` voor details.
