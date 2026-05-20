---
description: "combineLatestAll is een operator die een hogere-orde Observable (Observable of Observable) neemt en de laatste waarde van elk combineert zodra alle interne Observable ten minste één keer zijn afgevuurd."
---

# combineLatestAll - combineer de laatste waarden van de interne Observable

De `combineLatestAll` operator neemt een **Higher-order Observable** (Observable of Observables),
**Zodra alle interne Observable ten minste één keer zijn afgevuurd**, worden de **laatste waarden** van elk gecombineerd en uitgevoerd als een **array**.

## 🔰 Basissyntaxis en gebruik

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// 3Twee interneObservablemetHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Alle interneObservablehebben ten minste1een keer afgevuurd, combineer de laatste waarden
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Uitgang:
// [1, 3, 0] ← Als alles minstens één keer is afgevuurd1Als alles minstens één keer is afgevuurd (na2seconden later)
// [2, 3, 0] ← 1Wanneer de tweedeObservablewordt afgevuurd (na 2 seconden)2vuurt (na3seconden later)
// [2, 3, 1] ← 3Wanneer de tweedeObservablewordt afgevuurd (na 2 seconden)1vuurt (na4seconden later)
```

- Verzamel interne Observables wanneer Observable van hogere orde **compleet** is.
- Begin met combineren als alle interne Observable ten minste één keer zijn geactiveerd**.
- Telkens wanneer een Interne Observable een waarde afgeeft, **combineer** alle laatste waarden en **uitvoer**.

[🌐 Officiële RxJS documentatie - ` combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Typisch gebruikspatroon.

- Combineer de laatste resultaten van meerdere API-aanroepen**
- **Synchroniseer de laatste waarden van meerdere formulierinvoeren**
- **Integreer meerdere real-time gegevensbronnen**

## Praktische codevoorbeelden

Voorbeeld van het combineren van de laatste resultaten van meerdere API-aanroepen

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3Twee gesimuleerdeAPICreëer aanroep (Higher-order Observable)
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: Gebruikersinformatie (1Voltooid in seconden,3eenmaal bijgewerkt)
  timer(0, 1000).pipe(
    take(3),
    map(n => `Gebruiker: User${n}`)
  ),
  // API 2: Aantal meldingen (0.5Voltooid in seconden,4eenmaal bijgewerkt)
  timer(0, 500).pipe(
    take(4),
    map(n => `Meldingen: ${n}Aantal meldingen`)
  ),
  // API 3: Status2Voltooid in seconden,2eenmaal bijgewerkt)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Status: Offline' : 'Status: Online')
  )
);

// AlleAPIGecombineerde laatste waarde van oproepen
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>Laatste status:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- Drie API-oproepen worden parallel uitgevoerd.
- **Als ze allemaal ten minste één keer zijn uitgevoerd**, worden de gecombineerde resultaten weergegeven.
- Telkens wanneer een API wordt bijgewerkt, wordt de **laatste combinatie** weergegeven

## 🔄 Gerelateerde Creation Function

combineLatestAll` wordt echter voornamelijk gebruikt voor het afvlakken van hogere-orde Observable,
Gebruik de **Creation Function** `combineLatest` voor normale combinaties van Observables.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation FunctionUitgave (meer algemeen gebruik)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Zie [Hoofdstuk 3 Creation Function - combineLatest] (/guide/creation-functions/combination/combineLatest).

## 🔄 Verwante operatoren.

| Exploitant. | Beschrijving. |
|---|---|
| [mergeAll](./mergeAll) | Abonneer alle interne Observable parallel. |
| [concatAll](./concatAll) | Abonneer op interne Observable in volgorde. |
| [switchAll](./switchAll) | Schakel naar een nieuwe interne Observable. |
| [zipAll](./zipAll) | Koppel de waarden van elke interne Observable in de overeenkomstige volgorde |

## ⚠️ Opmerkingen.

### Observable van hogere orde moet worden ingevuld.

combineLatestAll` wacht op de verzameling van interne Observables **tot** de hogere-orde Observable (buitenste Observable) is **voltooid**.

#### ❌ De hogere-orde Observable wordt niet voltooid, dus er wordt niets uitgevoerd.

```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Er wordt niets uitgevoerd
```

#### ✅ Take to complete

```ts
interval(1000).pipe(
  take(3), // 3Voltooid in één sessie
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Alle interne Observable moeten ten minste één keer afgaan

Er wordt geen waarde uitgevoerd totdat alle interne Observable **ten minste eenmaal** zijn geactiveerd.

```ts
// 1Als een van de interneObservableEr wordt niets uitgevoerd als er
of(
  of(1, 2, 3),
  NEVER // Nooit voor altijd afgaat.
).pipe(
  combineLatestAll()
).subscribe(console.log); // Er wordt niets uitgevoerd
```

### Geheugengebruik.

Let op het geheugengebruik als er veel interne Observable zijn, aangezien de **laatste waarden van alle interne Observable in het geheugen worden bewaard**.
