---
description: De ignoreElements operator is een RxJS filteroperator die alle waarden negeert en alleen voltooiings- en foutmeldingen doorlaat. Het is nuttig bij het wachten op procesvoltooiing.
titleTemplate: ':title'
---

# ignoreElements - Negeer alle waarden en laat alleen voltooiing/fout door

De `ignoreElements` operator negeert **alle waarden** die door de bron Observable worden uitgegeven en laat **alleen voltooiings- en foutmeldingen** stroomafwaarts door.

## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Waarde:', value), // Wordt niet aangeroepen
  complete: () => console.log('Voltooid')
});
// Output: Voltooid
```

**Werkingsstroom**:
1. 1, 2, 3, 4, 5 worden allemaal genegeerd
2. Alleen voltooiingsmelding wordt stroomafwaarts doorgegeven

[🌐 RxJS Officiële Documentatie - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Typische gebruikspatronen

- **Wachten op procesvoltooiing**: Wanneer waarden onnodig zijn en alleen voltooiing nodig is
- **Alleen neveneffecten uitvoeren**: Neveneffecten uitvoeren met tap en waarden negeren
- **Foutafhandeling**: Wanneer u alleen fouten wilt vangen
- **Sequentiesynchronisatie**: Wacht op voltooiing van meerdere processen

## 🧠 Praktisch codevoorbeeld 1: Wachten op initialisatievoltooiing

Voorbeeld van wachten tot meerdere initialisatieprocessen voltooid zijn.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// Maak UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Applicatie-initialisatie';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Functie om statuslog toe te voegen
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Initialisatieproces 1: Databaseverbinding
const initDatabase$ = from(['Verbinden met DB...', 'Tabellen controleren...', 'DB gereed']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Negeer waarden, meld alleen voltooiing
);

// Initialisatieproces 2: Configuratiebestand laden
const loadConfig$ = from(['Configuratiebestand laden...', 'Configuratie parsen...', 'Configuratie toegepast']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Initialisatieproces 3: Gebruikersauthenticatie
const authenticate$ = from(['Inloggegevens controleren...', 'Token valideren...', 'Authenticatie voltooid']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Voer alle initialisatieprocessen uit
addLog('Initialisatie gestart...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ Alle initialisatie voltooid! Applicatie kan starten.';
    addLog('Applicatie lancering', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Initialisatiefout: ${err.message}`;
  }
});
```

- Gedetailleerde logs voor elk initialisatieproces worden weergegeven, maar waarden worden genegeerd.
- Een voltooiingsbericht wordt weergegeven wanneer alle processen voltooid zijn.

## 🆚 Vergelijking met vergelijkbare operators

### ignoreElements vs filter(() => false) vs take(0)

| Operator | Waardeverwerking | Voltooiingsmelding | Gebruiksscenario |
|:---|:---|:---|:---|
| `ignoreElements()` | Negeer alles | Laat door | **Alleen voltooiing nodig** (aanbevolen) |
| `filter(() => false)` | Filter alles | Laat door | Voorwaardelijke filtering (toevallig allemaal uitgesloten) |
| `take(0)` | Voltooi onmiddellijk | Laat door | Wil onmiddellijk voltooien |

**Aanbevolen**: Gebruik `ignoreElements()` wanneer u opzettelijk alle waarden negeert. Het maakt de code-intentie duidelijk.

## 🔄 Afhandeling van foutmeldingen

`ignoreElements` negeert waarden maar **laat foutmeldingen door**.

```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Fout opgetreden'))
).pipe(
  ignoreElements()
);

// Successcenario
success$.subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('✅ Voltooid'),
  error: err => console.error('❌ Fout:', err.message)
});
// Output: ✅ Voltooid

// Foutscenario
error$.subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('✅ Voltooid'),
  error: err => console.error('❌ Fout:', err.message)
});
// Output: ❌ Fout: Fout opgetreden
```

## ⚠️ Opmerkingen

### 1. Neveneffecten worden uitgevoerd

`ignoreElements` negeert waarden maar neveneffecten (zoals `tap`) worden uitgevoerd.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Neveneffect:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('Voltooid')
});
// Output:
// Neveneffect: 1
// Neveneffect: 2
// Neveneffect: 3
// Voltooid
```

### 2. Gebruik met oneindige Observables

Met oneindige Observables gaat het abonnement voor altijd door omdat voltooiing nooit komt.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Slecht voorbeeld: Voltooit niet
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Voltooid') // Wordt niet aangeroepen
});

// ✅ Goed voorbeeld: Voltooi met take
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Voltooid') // Wordt na 5 seconden aangeroepen
});
```

### 3. TypeScript type

De retourwaarde van `ignoreElements` is van type `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// Resultaat van ignoreElements is Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value is never type, dus dit blok wordt niet uitgevoerd
    console.log(value);
  },
  complete: () => console.log('Alleen voltooiing')
});
```

## 📚 Gerelateerde operators

- **[filter](/nl/guide/operators/filtering/filter)** - Filter waarden op basis van voorwaarden
- **[take](/nl/guide/operators/filtering/take)** - Haal alleen eerste N waarden
- **[skip](/nl/guide/operators/filtering/skip)** - Sla eerste N waarden over
- **[tap](https://rxjs.dev/api/operators/tap)** - Voer neveneffecten uit (officiële documentatie)

## Samenvatting

De `ignoreElements` operator negeert alle waarden en laat alleen voltooiing en fout door.

- ✅ Ideaal wanneer alleen voltooiingsmelding nodig is
- ✅ Neveneffecten (tap) worden uitgevoerd
- ✅ Laat ook foutmeldingen door
- ✅ Intentie duidelijker dan `filter(() => false)`
- ⚠️ Voltooit niet met oneindige Observables
- ⚠️ Retourwaarde type is `Observable<never>`
- ⚠️ Waarden worden volledig genegeerd maar neveneffecten worden uitgevoerd
