---
description: "De elementAt operator is een RxJS filteroperator die alleen waarden op een bepaalde indexpositie ophaalt. Het werkt vergelijkbaar met array index toegang."
---

# elementAt - opgehaald door indexspecificatie

De `elementAt` operator haalt **alleen de waarde op de opgegeven indexpositie** op uit de Observable en voltooit de stroom onmiddellijk. Het werkt hetzelfde als `array[index]` van een array.

## 🔰 Basissyntaxis en gebruik

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Uitvoer.: 30(Index2Waarde)
```

**Bewerkingsstroom**:.
1. 10 (index 0) → overslaan
2. 20 (index 1) → overslaan
3. 30 (index 2) → uitvoer en voltooien
4. 40, 50 niet geëvalueerd

[🌐 Officiële RxJS documentatie - elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Typisch gebruikspatroon.

- **Paginatie**: het eerste item op een specifieke pagina ophalen.
- **Op volgorde gegarandeerde gegevens verkrijgen**: de N-de gebeurtenis of het N-de bericht verkrijgen.
- **Testen en debuggen**: valideer de waarde van een specifieke positie.
- **Array-achtige toegang**: behandel Observable als een array.

## Praktisch codevoorbeeld 1: Aftellen van gebeurtenissen

Dit is een voorbeeld van het uitvoeren van een actie bij de N-de klik.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UIMaak
const output = document.createElement('div');
output.innerHTML = '<h3>5Klik eenmaal om het bericht weer te geven</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Klik op';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'meer5Eenmaal klikken';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Klik gebeurtenis
const clicks$ = fromEvent(button, 'click');

// Voor telweergave
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `meer${remaining}Eenmaal klikken`;
  } else {
    counter.textContent = '';
  }
});

// 5Tweede keer (index)4Gedetecteerde klikken van
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Bereikt！';
  result.style.color = 'green';
  button.disabled = true;
});
```

- De vijfde klik (index 4) voltooit de actie.
- Het begint bij 0, net als de array-index.

## Praktisch codevoorbeeld 2: Haal het N-de getal uit de gegevensstroom.

Dit is een voorbeeld van het ophalen van een specifieke volgorde van waarden uit gegevens die met regelmatige tussenpozen worden gepubliceerd.

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UIMaak
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Uit de gegevensstroomNVerkrijg de tweede';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Voer de index in (0〜uit de gegevensstroom (9)';
input.min = '0';
input.max = '9';
input.style.marginRight = '10px';
container.appendChild(input);

const getButton = document.createElement('button');
getButton.textContent = 'Ophalen';
container.appendChild(getButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Gegevensstroom (0.5De waarden worden elke seconde uitgegeven,10maximaal 1)
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = '0〜uit de gegevensstroom (9Voer een bereik in van';
    status.style.color = 'red';
    return;
  }

  status.textContent = `Index ${index} Waarde wordt opgehaald...`;
  status.style.color = 'blue';
  result.style.display = 'none';
  getButton.disabled = true;
  input.disabled = true;

  data$.pipe(
    elementAt(index)
  ).subscribe({
    next: data => {
      status.textContent = '';
      result.style.display = 'block';
      result.innerHTML = `
        <strong>✅ Succesvol ophalen</strong><br>
        Index: ${data.index}<br>
        Waarde: ${data.value}<br>
        Tijdstempel: ${new Date(data.timestamp).toLocaleTimeString()}
      `;
      result.style.color = 'green';
      result.style.backgroundColor = '#e8f5e9';
      getButton.disabled = false;
      input.disabled = false;
    },
    error: err => {
      status.textContent = '';
      result.style.display = 'block';
      result.textContent = `❌ Fout: ${err.message}`;
      result.style.color = 'red';
      result.style.backgroundColor = '#ffebee';
      getButton.disabled = false;
      input.disabled = false;
    }
  });
});
```

- Haalt waarden op bij een opgegeven index uit een stroom die elke 0,5 seconden wordt uitgegeven.
- Er wordt een fout gegenereerd als de index buiten bereik is.

## Vergelijking met vergelijkbare operatoren

### elementAt vs take vs first

```ts
import { from } from 'rxjs';
import { elementAt, take, first, skip } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// elementAt: Alleen waarden op een specifieke index worden opgehaald
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Uitvoer.: 30

// take: Vanaf het beginNEén waarde vanaf het begin ophalen
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Uitvoer.: 10, 20, 30

// skip + first: elementAt Gelijk aan (overbodig)
numbers$.pipe(
  skip(2),
  first()
).subscribe(console.log);
// Uitvoer.: 30
```

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Uitvoer.: 30(Index2Waarde)

## ⚠️ Opmerkingen.

### 1. als de index buiten bereik is

Als de opgegeven index niet is bereikt voordat de stream is voltooid, wordt er een fout gegenereerd.

```

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // 3Slechts één

numbers$.pipe(
  elementAt(5) // Index5Verzoek
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Uitvoer.: Fout: no elements in sequence
```

### 2. Geef standaardwaarden op.

Om fouten te voorkomen, kunnen standaardwaarden worden opgegeven.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Geef een standaardwaarde op
numbers$.pipe(
  elementAt(5, 999) // Index5Indien niet aanwezig, retourneert999Geeft een
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Uitvoer.: 999
```

### 3. Gebruik met asynchrone streams

Wacht in asynchrone streams tot de indexpositie is bereikt.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// 1Geeft elke seconde een waarde
interval(1000).pipe(
  elementAt(3) // Index3(4(tweede waarde)
).subscribe(console.log);
// 3Uitvoer na seconden: 3
```

### 4. Negatieve indexen zijn niet toegestaan

Negatieve indexen kunnen niet worden opgegeven.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ Negatieve indexen zijn fouten
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Fout: ArgumentOutOfRangeError: index out of range
```

Gebruik `takeLast` of `last` om aan het einde van de array te komen.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Laatste waarde krijgen
numbers$.pipe(
  last()
).subscribe(console.log);
// Uitvoer.: 50

// ✅ LaatsteNLaatste waarde
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Uitvoer.: 40, 50
```

## 📚 Verwante operatoren.

- **[take](. /take)** - N genomen vanaf het begin.
- **[first](. /first)** - eerste waarde krijgen.
- **[last](. /last)** - laatste waarde krijgen.
- **[skip](. /skip)** - de eerste N waarden overslaan
- **[takeLast](. /takeLast)** - de laatste N waarden krijgen

## Samenvatting.

De `elementAt` operator haalt alleen de waarde op de opgegeven indexpositie op.

- ✅ Hetzelfde gedrag als array index toegang.
- ✅ Ideaal voor het ophalen van de N-de waarde
- Standaardwaarden kunnen gespecificeerd worden om fouten te vermijden
- ⚠️ Fout als index buiten bereik is (geen standaardwaarde)
- ⚠️ Negatieve indexen zijn niet toegestaan
- ⚠️ Asynchrone streams wachten tot ze bereikt zijn
