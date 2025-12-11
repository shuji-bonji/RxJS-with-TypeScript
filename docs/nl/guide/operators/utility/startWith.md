---
description: De startWith operator voegt de gespecificeerde beginwaarde in voordat de Observable de waarde uitzendt, en is geschikt voor status-initialisatie en initiële weergave van de UI.
---

# startWith - Geef beginwaarde

De `startWith` operator is een operator om **de gespecificeerde beginwaarde uit te zenden voordat de bron Observable de waarde uitzendt**.
Het wordt gebruikt voor statusbeheer, initiële weergave, plaatshouderswaarden, etc.


## 🔰 Basissyntax en werking

```ts
import { of } from 'rxjs';
import { startWith } from 'rxjs';

of('B', 'C').pipe(
  startWith('A')
).subscribe(console.log);
// Uitvoer:
// A
// B
// C
```

Zo voegt `startWith` eerst `'A'` toe, gevolgd door de waarden van de bron Observable.

[🌐 RxJS Officiële Documentatie - startWith](https://rxjs.dev/api/index/function/startWith)

## 💡 Typisch gebruiksvoorbeeld

Dit is nuttig wanneer u beginwaarden voor statussen of tellers wilt instellen. Hier is een voorbeeld van een teller die start met een beginwaarde van `100`.

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

interval(1000)
  .pipe(
    startWith(-1), // Voeg eerst -1 in
    scan((acc, curr) => acc + 1, 100), // Verhoog vanaf beginwaarde 100
    take(10) // Geef 10 keer totaal uit
  )
  .subscribe(console.log);
// Uitvoer:
// 101
// 102
// 103
// 104
// 105
// 106
// 107
// 108
// 109
// 110
```


## 🧪 Praktisch codevoorbeeld (met UI)

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

// Uitvoerweergavegebied
const startWithOutput = document.createElement('div');
startWithOutput.innerHTML = '<h3>startWith voorbeeld:</h3>';
document.body.appendChild(startWithOutput);

// Tellerweergavegebied
const counterDisplay = document.createElement('div');
counterDisplay.style.fontSize = '24px';
counterDisplay.style.fontWeight = 'bold';
counterDisplay.style.textAlign = 'center';
counterDisplay.style.padding = '20px';
counterDisplay.style.border = '1px solid #ddd';
counterDisplay.style.borderRadius = '5px';
counterDisplay.style.margin = '10px 0';
startWithOutput.appendChild(counterDisplay);

// Waardenlijst weergavegebied
const valuesList = document.createElement('div');
valuesList.style.marginTop = '10px';
startWithOutput.appendChild(valuesList);

// Tellerstream (elke 1 seconde)
interval(1000)
  .pipe(
    // Start eerst met 100
    startWith(-1),
    // Voeg 1 toe aan elke waarde bij de vorige waarde
    scan((acc, curr) => acc + 1, 100),
    // Eindig na 10 keer
    take(10)
  )
  .subscribe((count) => {
    // Update tellerweergave
    counterDisplay.textContent = count.toString();

    // Voeg waarde toe aan lijst
    const valueItem = document.createElement('div');

    if (count === 100) {
      valueItem.textContent = `Beginwaarde: ${count} (toegevoegd met startWith)`;
      valueItem.style.color = 'blue';
    } else {
      valueItem.textContent = `Volgende waarde: ${count}`;
    }

    valuesList.appendChild(valueItem);
  });
```


## ✅ Samenvatting

- `startWith` is nuttig voor situaties waar u **een vaste waarde als eerste wilt invoegen**
- Veelgebruikt voor status-initialisatie, UI-plaatshouders, initiële weergave van formulieren, etc.
- Gebruikt in combinatie met `scan`, `combineLatest`, etc. om **de basis voor statusbeheer te bouwen**
