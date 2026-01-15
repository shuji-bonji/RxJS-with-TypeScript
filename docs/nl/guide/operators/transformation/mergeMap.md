---
description: De mergeMap operator converteert elke waarde naar een nieuwe Observable, voert ze gelijktijdig uit en combineert ze plat. Dit is nuttig wanneer meerdere API-verzoeken parallel moeten worden uitgevoerd zonder sequentieel te wachten, of om geneste asynchrone verwerking te beheren.
titleTemplate: ':title'
---

# mergeMap - Parallelle samenvoeging

De `mergeMap` (ook bekend als `flatMap`) operator converteert elke waarde naar een nieuwe Observable en **voegt ze plat gelijktijdig samen**.
Het is zeer nuttig wanneer u verzoeken onmiddellijk wilt uitvoeren zonder sequentieel te wachten, of voor geneste asynchrone verwerking.

## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  mergeMap(value =>
    of(`${value} voltooid`).pipe(delay(1000))
  )
).subscribe(console.log);

// Output voorbeeld (in willekeurige volgorde):
// A voltooid
// B voltooid
// C voltooid
```

- Genereert een nieuwe Observable voor elke waarde.
- Die Observables worden **parallel uitgevoerd** en de resultaten worden in willekeurige volgorde uitgevoerd.

[🌐 RxJS Officiële Documentatie - `mergeMap`](https://rxjs.dev/api/operators/mergeMap)

## 💡 Typische gebruikspatronen

- Verstuur API-verzoek voor elke knopklik
- Start bestandsupload voor elke file drop-gebeurtenis
- Activeer asynchrone taken gelijktijdig getriggerd door gebruikersacties

## 🧠 Praktisch codevoorbeeld (met UI)

Dit is een voorbeeld van het triggeren van een asynchroon verzoek (respons na 2 seconden) elke keer dat een knop wordt geklikt.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

// Maak knop
const button = document.createElement('button');
button.textContent = 'Verstuur verzoek';
document.body.appendChild(button);

// Uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Klikgebeurtenis
fromEvent(button, 'click').pipe(
  mergeMap((_, index) => {
    const requestId = index + 1;
    console.log(`Verzoek ${requestId} gestart`);
    return of(`Respons ${requestId}`).pipe(delay(2000));
  })
).subscribe((response) => {
  const div = document.createElement('div');
  div.textContent = `✅ ${response}`;
  output.appendChild(div);
});
```

- Met elke klik wordt onmiddellijk een asynchroon verzoek verstuurd.
- **Wacht 2 seconden voor elk verzoek individueel**, dus resultaten zijn niet geordend op volgorde van aankomst.
- Dit is een geweldig voorbeeld voor het begrijpen van parallelle verwerking.
