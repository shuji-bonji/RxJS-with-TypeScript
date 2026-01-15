---
description: "exhaustAll operator negeert nieuwe interne Observables terwijl er een actief is: Essentieel voor het voorkomen van dubbele klikken, dubbele inzendingen en knop-mashing"
titleTemplate: ':title | RxJS'
---

# exhaustAll - Negeer Tijdens Actief

De `exhaustAll` operator neemt een **Higher-order Observable** (Observable van Observables),
**negeert nieuwe interne Observables** als er een interne Observable actief is.

## 🔰 Basissyntax en gebruik

```ts
import { fromEvent, interval } from 'rxjs';
import { map, exhaustAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Start een nieuwe teller voor elke klik (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Negeer nieuwe klikken als teller actief is
higherOrder$
  .pipe(exhaustAll())
  .subscribe(x => console.log(x));

// Output (met 3 opeenvolgende klikken):
// 0 (1e teller)
// 1 (1e teller)
// ← Klik hier (genegeerd: 1e is actief)
// 2 (1e teller) ← Voltooid
// ← Klik hier (geaccepteerd: geen actieve teller)
// 0 (2e teller)
// 1 (2e teller)
// 2 (2e teller)
```

- Als interne Observable actief is, worden **nieuwe interne Observables genegeerd**
- **Accepteert volgende na** voltooiing van actieve Observable
- Ideaal voor het voorkomen van dubbele uitvoering

[🌐 RxJS Officiële Documentatie - `exhaustAll`](https://rxjs.dev/api/index/function/exhaustAll)

## 💡 Typische gebruikspatronen

- **Dubbele-klik preventie (voorkom knop-mashing)**
- **Voorkom dubbele loginverzoeken**
- **Voorkom dubbele opslagoperaties**

## 🧠 Praktisch codevoorbeeld

Voorbeeld van het voorkomen van dubbele klikken op opslagknop

```ts
import { fromEvent, of } from 'rxjs';
import { map, exhaustAll, delay } from 'rxjs';

const saveButton = document.createElement('button');
saveButton.textContent = 'Opslaan';
document.body.appendChild(saveButton);

const output = document.createElement('div');
document.body.appendChild(output);

let saveCount = 0;

// Knopklikgebeurtenis
const clicks$ = fromEvent(saveButton, 'click');

// Higher-order Observable: Gesimuleerde opslagoperatie voor elke klik
const saves$ = clicks$.pipe(
  map(() => {
    const id = ++saveCount;
    const start = Date.now();

    // Tijdelijk knop uitschakelen (visuele feedback)
    saveButton.disabled = true;

    // Gesimuleerde opslagoperatie (2 seconden vertraging)
    return of(`Opslaan voltooid #${id}`).pipe(
      delay(2000),
      map(msg => {
        saveButton.disabled = false;
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} seconden)`;
      })
    );
  }),
  exhaustAll() // Negeer nieuwe klikken tijdens opslaan
);

saves$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});

// Log genegeerde klikken
clicks$.subscribe(() => {
  if (saveButton.disabled) {
    console.log('Klik genegeerd tijdens opslagoperatie');
  }
});
```

- **Nieuwe klikken worden genegeerd** tijdens opslagoperatie
- Volgende klik wordt geaccepteerd na voltooiing van opslaan

## 🔄 Gerelateerde operators

| Operator | Beschrijving |
|---|---|
| `exhaustMap` | Afkorting voor `map` + `exhaustAll` (vaak gebruikt) |
| [mergeAll](/nl/guide/operators/combination/mergeAll) | Abonneer op alle interne Observables parallel |
| [concatAll](/nl/guide/operators/combination/concatAll) | Abonneer op interne Observables in volgorde (wachtrij) |
| [switchAll](/nl/guide/operators/combination/switchAll) | Schakel naar nieuwe interne Observable (annuleer oude) |

## 🔄 Vergelijking met andere operators

| Operator | Wanneer nieuwe interne Observable wordt geëmitteerd |
|---|---|
| `mergeAll` | Gelijktijdig uitvoeren |
| `concatAll` | Toevoegen aan wachtrij (wacht op vorige voltooiing) |
| `switchAll` | Annuleer oude en schakel |
| `exhaustAll` | **Negeer (wacht op voltooiing van actieve)** |

## ⚠️ Belangrijke opmerkingen

### Gebeurtenisverlies

`exhaustAll` **negeert volledig** actieve gebeurtenissen, dus het is ongeschikt als u alle gebeurtenissen wilt verwerken.

```ts
// ❌ exhaustAll is ongeschikt als u alle klikken wilt registreren
// ✅ Gebruik mergeAll of concatAll
```

### UI Feedback

Het is belangrijk om gebruikers visueel te vertellen dat gebeurtenissen worden "genegeerd".

```ts
// Schakel knop uit
saveButton.disabled = true;

// Toon toast-bericht
showToast('Aan het verwerken. Even geduld.');
```

### Geschikte use cases

#### `exhaustAll` is optimaal voor:
- Loginoperaties (voorkom dubbele inzendingen)
- Opslagoperaties (voorkom dubbele uitvoering)
- Animaties (start geen nieuwe animatie terwijl er een actief is)

#### `exhaustAll` is niet geschikt voor:
- Zoekoperaties (wil laatste zoekopdracht uitvoeren → `switchAll`)
- Alle gebeurtenissen moeten worden verwerkt (→ `mergeAll` of `concatAll`)
