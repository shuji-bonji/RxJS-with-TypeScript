---
description: raceWith is een RxJS combinatie-operator die alleen de eerste stream adopteert die een waarde emitteert onder de originele Observable en andere Observables. Het is de Pipeable Operator versie van Creation Function race, en is ideaal voor situaties waar de snelste respons prioriteit heeft, zoals timeout-implementaties, parallelle acquisitie van meerdere CDN's (fallback), en concurrerende acquisitie van meerdere databronnen.
titleTemplate: ':title | RxJS'
---

# raceWith - Adopteer de Snelste Stream (Binnen Pipeline)

De `raceWith` operator **adopteert alleen de eerste stream die een waarde emitteert** onder de originele Observable en de gespecificeerde andere Observables, en negeert alle anderen.
Dit is de Pipeable Operator versie van de Creation Function `race`.

## 🔰 Basissyntax en gebruik

```ts
import { interval, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  take(3),
  map(val => `Bron 1: ${val}`)
);

const source2$ = timer(500).pipe(
  take(3),
  map(val => `Bron 2: ${val}`)
);

source1$
  .pipe(raceWith(source2$))
  .subscribe(console.log);

// Output:
// Bron 2: 0 (na 500ms)
// * source1$ wordt genegeerd omdat source2$ eerst emitteerde
```

- **De eerste Observable die een waarde emitteert** wint de race, en alleen die stream wordt geadopteerd.
- Andere Observables worden automatisch uitgeschreven en genegeerd.

[🌐 RxJS Officiële Documentatie - `raceWith`](https://rxjs.dev/api/operators/raceWith)


## 💡 Typische gebruikspatronen

- **Timeout-implementatie**: Laat hoofdverwerking concurreren met timeout-fout na een bepaalde tijd
- **Parallelle acquisitie van meerdere CDN's**: Verzoek meerdere CDN's gelijktijdig en adopteer de snelste respons (fallback-strategie)
- **Concurrerende acquisitie van meerdere databronnen**: Voer lokale cache en API-aanroep gelijktijdig uit, en gebruik degene die het eerst terugkeert
- **Gebruikersactie vs timer competitie**: Laat klikactie concurreren met automatisch doorschakelen, en adopteer wat het eerst gebeurt


## 🧠 Praktisch codevoorbeeld (met UI)

Voorbeeld van het ophalen van data van meerdere CDN's parallel en het adopteren van de snelste respons.

```ts
import { fromFetch } from 'rxjs/fetch';
import { raceWith, map, catchError, timeout } from 'rxjs';
import { of } from 'rxjs';

// Bouw de UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>raceWith Praktisch Voorbeeld: Parallel Ophalen van Meerdere CDN's</h3>
  <button id="fetch-button">Data Ophalen</button>
  <div id="status" style="margin-top: 10px; padding: 10px; border: 1px solid #ccc;">
    Wachten...
  </div>
  <div id="result" style="margin-top: 10px;"></div>
`;
document.body.appendChild(container);

const fetchButton = document.getElementById('fetch-button') as HTMLButtonElement;
const statusDiv = document.getElementById('status')!;
const resultDiv = document.getElementById('result')!;

// Start data ophalen bij klik op knop
fetchButton.addEventListener('click', () => {
  statusDiv.textContent = 'Data ophalen van meerdere CDN\'s parallel...';
  statusDiv.style.backgroundColor = '#fff3e0';
  resultDiv.innerHTML = '';

  // Meerdere CDN's (eigenlijk dummy endpoints)
  const cdn1$ = fromFetch('https://jsonplaceholder.typicode.com/posts/1').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 1', data: 'Data succesvol opgehaald' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 1', data: 'Fout' }))
  );

  const cdn2$ = fromFetch('https://jsonplaceholder.typicode.com/posts/2').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 2', data: 'Data succesvol opgehaald' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 2', data: 'Fout' }))
  );

  const cdn3$ = fromFetch('https://jsonplaceholder.typicode.com/posts/3').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 3', data: 'Data succesvol opgehaald' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 3', data: 'Fout' }))
  );

  // ✅ Adopteer snelste respons met raceWith
  cdn1$
    .pipe(raceWith(cdn2$, cdn3$))
    .subscribe({
      next: (result) => {
        statusDiv.textContent = `✅ Succesvol opgehaald van ${result.source}`;
        statusDiv.style.backgroundColor = '#e8f5e9';
        resultDiv.innerHTML = `<strong>${result.source}</strong>: ${result.data}`;
      },
      error: (err) => {
        statusDiv.textContent = '❌ Ophalen van alle CDN\'s mislukt';
        statusDiv.style.backgroundColor = '#ffebee';
        resultDiv.textContent = `Fout: ${err.message}`;
      }
    });
});
```

- Verzoekt meerdere CDN's gelijktijdig, en **adopteert de eerste CDN** die een respons retourneert.
- Responsen van andere CDN's worden automatisch genegeerd.


## 🔄 Verschil met Creation Function `race`

### Basisverschillen

| | `race` (Creation Function) | `raceWith` (Pipeable Operator) |
|:---|:---|:---|
| **Gebruikslocatie** | Gebruikt als onafhankelijke functie | Gebruikt binnen `.pipe()` keten |
| **Syntax** | `race(obs1$, obs2$, obs3$)` | `obs1$.pipe(raceWith(obs2$, obs3$))` |
| **Eerste stream** | Behandelt allemaal gelijk | Behandelt als hoofdstream |
| **Voordeel** | Eenvoudig en leesbaar | Eenvoudig te combineren met andere operators |

### Specifieke gebruiksvoorbeelden

**Creation Function wordt aanbevolen voor alleen eenvoudige competitie**

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const fast$ = timer(100).pipe(map(() => 'Snel wint!'));
const slow$ = timer(500).pipe(map(() => 'Langzaam wint!'));

// Eenvoudig en leesbaar
race(fast$, slow$).subscribe(console.log);
// Output: Snel wint!
```

**Pipeable Operator wordt aanbevolen bij het toevoegen van transformatieverwerking aan hoofdstream**

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, mapTo, take } from 'rxjs';

// Gebruikersklik vs automatisch doorschakelen competitie
const userClick$ = fromEvent(document, 'click').pipe(
  take(1),
  mapTo('Gebruiker klikte')
);

const autoAdvance$ = timer(5000).pipe(
  mapTo('Automatisch doorgeschakeld')
);

// ✅ Pipeable Operator versie - voeg verwerking toe aan hoofdstream
userClick$
  .pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`),
    raceWith(autoAdvance$.pipe(
      map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
    ))
  )
  .subscribe(console.log);

// ❌ Creation Function versie - wordt omslachtig
import { race } from 'rxjs';
race(
  userClick$.pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
  ),
  autoAdvance$.pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
  )
).subscribe(console.log);
```

### Samenvatting

- **`race`**: Optimaal voor eenvoudig laten concurreren van meerdere streams
- **`raceWith`**: Optimaal wanneer u met andere streams wilt concurreren terwijl u de hoofdstream transformeert of verwerkt


## ⚠️ Belangrijke opmerkingen

### Eerste emissie wint

De stream met de **vroegste emissietiming** wordt geadopteerd. Niet de starttiming van abonnement.

```ts
import { timer, of } from 'rxjs';
import { raceWith, map } from 'rxjs';

const immediate$ = of('Emitteer onmiddellijk');
const delayed$ = timer(1000).pipe(map(() => 'Emitteer na 1 seconde'));

immediate$
  .pipe(raceWith(delayed$))
  .subscribe(console.log);
// Output: Emitteer onmiddellijk
```

### Alle Observables worden geabonneerd

`raceWith` **abonneert op alle Observables tegelijkertijd**, maar negeert alle behalve de eerste die emitteert.

```ts
import { timer } from 'rxjs';
import { raceWith, tap } from 'rxjs';

const source1$ = timer(100).pipe(
  tap(() => console.log('Bron 1 emitteert'))
);

const source2$ = timer(200).pipe(
  tap(() => console.log('Bron 2 emitteert'))
);

source1$
  .pipe(raceWith(source2$))
  .subscribe(console.log);
// Output:
// Bron 1 emitteert
// 0
// Bron 2 emitteert ← Geabonneerd, maar waarde wordt genegeerd
```

### Foutafhandeling

Als er een Observable is die eerst een fout geeft, eindigt de gehele stream met een fout.

```ts
import { throwError, timer } from 'rxjs';
import { raceWith, catchError } from 'rxjs';
import { of } from 'rxjs';

timer(1000).pipe(
  raceWith(
    throwError(() => new Error('Fout opgetreden')).pipe(
      catchError(err => of('Fout hersteld'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Output: Fout hersteld
```


## 📚 Gerelateerde operators

- **[race](/nl/guide/creation-functions/selection/race)** - Creation Function versie
- **[mergeWith](/nl/guide/operators/combination/mergeWith)** - Voer alle streams parallel uit
- **[concatWith](/nl/guide/operators/combination/concatWith)** - Voer streams sequentieel uit
