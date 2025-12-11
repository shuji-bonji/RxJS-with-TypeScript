---
description: De elementAt operator is een RxJS filteroperator die alleen de waarde op een gespecificeerde indexpositie ophaalt. Het gedraagt zich vergelijkbaar met array-index toegang.
---

# elementAt - Haal waarde op gespecificeerde index

De `elementAt` operator haalt **alleen de waarde op de gespecificeerde indexpositie** van een Observable op en voltooit de stream onmiddellijk. Het gedraagt zich vergelijkbaar met `array[index]`.

## 🔰 Basissyntax en gebruik

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30 (waarde op index 2)
```

**Werkingsstroom**:
1. 10 (index 0) → Overslaan
2. 20 (index 1) → Overslaan
3. 30 (index 2) → Uitvoeren en voltooien
4. 40, 50 worden niet geëvalueerd

[🌐 RxJS Officiële Documentatie - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Typische gebruikspatronen

- **Paginering**: Eerste item van een specifieke pagina ophalen
- **Geordende data-ophaling**: Nde gebeurtenis of bericht ophalen
- **Testen en debuggen**: Waarde op specifieke positie verifiëren
- **Array-achtige toegang**: Observable behandelen als een array

## 🧠 Praktisch codevoorbeeld: Gebeurtenisaftelling

Voorbeeld van het uitvoeren van een actie op de Nde klik.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// Maak UI
const output = document.createElement('div');
output.innerHTML = '<h3>Toon bericht bij 5e klik</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Klik';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Klik nog 5 keer';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Klikgebeurtenis
const clicks$ = fromEvent(button, 'click');

// Tellerweergave
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `Nog ${remaining} klikken`;
  } else {
    counter.textContent = '';
  }
});

// Detecteer 5e klik (index 4)
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Bereikt!';
  result.style.color = 'green';
  button.disabled = true;
});
```

- Voltooit bij de 5e klik (index 4).
- Begint bij 0, hetzelfde als array-index.

## 🆚 Vergelijking met vergelijkbare operators

### elementAt vs take vs first

| Operator | Opgehaalde waarde | Aantal uitvoer | Gebruiksscenario |
|:---|:---|:---|:---|
| `elementAt(n)` | Alleen waarde op index n | 1 | Nde waarde ophalen |
| `take(n)` | Eerste n waarden | n | Eerste N waarden ophalen |
| `first()` | Eerste waarde | 1 | Eerste ophalen |
| `skip(n) + first()` | Eerste na overslaan van n | 1 | Zelfde als elementAt (niet aanbevolen) |

## ⚠️ Opmerkingen

### 1. Wanneer index buiten bereik is

Als de gespecificeerde index niet wordt bereikt voordat de stream voltooit, treedt een fout op.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // Slechts 3 items

numbers$.pipe(
  elementAt(5) // Vraag index 5
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Output: Fout: no elements in sequence
```

### 2. Standaardwaarde specificeren

U kunt een standaardwaarde specificeren om fouten te voorkomen.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Specificeer standaardwaarde
numbers$.pipe(
  elementAt(5, 999) // Retourneer 999 als index 5 niet bestaat
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Output: 999
```

### 3. Gebruik met asynchrone streams

Voor asynchrone streams wacht het tot het de indexpositie bereikt.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// Emitteer waarde elke seconde
interval(1000).pipe(
  elementAt(3) // Index 3 (4e waarde)
).subscribe(console.log);
// Output na 3 seconden: 3
```

### 4. Negatieve index niet beschikbaar

Negatieve indexen kunnen niet worden gespecificeerd.

Om vanaf het einde van de array te verkrijgen, gebruik `takeLast` of `last`.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Haal laatste waarde
numbers$.pipe(
  last()
).subscribe(console.log);
// Output: 50

// ✅ Haal laatste N waarden
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Output: 40, 50
```

## 📚 Gerelateerde operators

- **[take](/nl/guide/operators/filtering/take)** - Haal eerste N waarden
- **[first](/nl/guide/operators/filtering/first)** - Haal eerste waarde
- **[last](/nl/guide/operators/filtering/last)** - Haal laatste waarde
- **[skip](/nl/guide/operators/filtering/skip)** - Sla eerste N waarden over
- **[takeLast](/nl/guide/operators/filtering/takeLast)** - Haal laatste N waarden

## Samenvatting

De `elementAt` operator haalt alleen de waarde op de gespecificeerde indexpositie op.

- ✅ Zelfde gedrag als array-index toegang
- ✅ Ideaal voor het ophalen van Nde waarde
- ✅ Kan fouten vermijden door standaardwaarde te specificeren
- ⚠️ Fout als index buiten bereik is (zonder standaardwaarde)
- ⚠️ Negatieve index niet beschikbaar
- ⚠️ Wacht tot positie bereikt voor asynchrone streams
