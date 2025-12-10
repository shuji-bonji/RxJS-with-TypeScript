---
description: "L'operatore exhaustAll ignora nuovi Observable interni mentre uno è in esecuzione: Essenziale per prevenire doppi click, invii duplicati e pressioni ripetute dei pulsanti"
titleTemplate: ':title | RxJS'
---

# exhaustAll - Ignora Nuovi Observable Interni Durante l'Esecuzione

L'operatore `exhaustAll` prende un **Higher-order Observable** (Observable di Observable),
**ignora nuovi Observable interni** se un Observable interno è in esecuzione.

## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent, interval } from 'rxjs';
import { map, exhaustAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Avvia un nuovo contatore per ogni click (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Ignora nuovi click se il contatore è in esecuzione
higherOrder$
  .pipe(exhaustAll())
  .subscribe(x => console.log(x));

// Output (con 3 click consecutivi):
// 0 (1° contatore)
// 1 (1° contatore)
// ← Click qui (ignorato: 1° è in esecuzione)
// 2 (1° contatore) ← Completo
// ← Click qui (accettato: nessun contatore in esecuzione)
// 0 (2° contatore)
// 1 (2° contatore)
// 2 (2° contatore)
```

- Se un Observable interno è in esecuzione, **i nuovi Observable interni vengono ignorati**
- **Accetta il successivo dopo che** l'Observable in esecuzione completa
- Ideale per prevenire la doppia esecuzione

[🌐 Documentazione Ufficiale RxJS - `exhaustAll`](https://rxjs.dev/api/index/function/exhaustAll)

## 💡 Pattern di Utilizzo Tipici

- **Prevenzione doppio click (prevenire pressioni ripetute dei pulsanti)**
- **Prevenire richieste di login duplicate**
- **Prevenire operazioni di salvataggio duplicate**

## 🧠 Esempio di Codice Pratico

Esempio di prevenzione doppi click sul pulsante di salvataggio

```ts
import { fromEvent, of } from 'rxjs';
import { map, exhaustAll, delay } from 'rxjs';

const saveButton = document.createElement('button');
saveButton.textContent = 'Salva';
document.body.appendChild(saveButton);

const output = document.createElement('div');
document.body.appendChild(output);

let saveCount = 0;

// Evento click del pulsante
const clicks$ = fromEvent(saveButton, 'click');

// Higher-order Observable: Operazione di salvataggio simulata per ogni click
const saves$ = clicks$.pipe(
  map(() => {
    const id = ++saveCount;
    const start = Date.now();

    // Disabilita temporaneamente il pulsante (feedback visivo)
    saveButton.disabled = true;

    // Operazione di salvataggio simulata (ritardo 2 secondi)
    return of(`Salvataggio completato #${id}`).pipe(
      delay(2000),
      map(msg => {
        saveButton.disabled = false;
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} secondi)`;
      })
    );
  }),
  exhaustAll() // Ignora nuovi click durante il salvataggio
);

saves$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});

// Log dei click ignorati
clicks$.subscribe(() => {
  if (saveButton.disabled) {
    console.log('Click ignorato durante l\'operazione di salvataggio');
  }
});
```

- **I nuovi click vengono ignorati** durante l'operazione di salvataggio
- Il click successivo viene accettato dopo che il salvataggio completa

## 🔄 Operatori Correlati

| Operatore | Descrizione |
|---|---|
| `exhaustMap` | Abbreviazione per `map` + `exhaustAll` (comunemente usato) |
| [mergeAll](/it/guide/operators/combination/mergeAll) | Sottoscrive tutti gli Observable interni in parallelo |
| [concatAll](/it/guide/operators/combination/concatAll) | Sottoscrive gli Observable interni in ordine (li mette in coda) |
| [switchAll](/it/guide/operators/combination/switchAll) | Passa al nuovo Observable interno (annulla il vecchio) |

## 🔄 Confronto con Altri Operatori

| Operatore | Quando Viene Emesso un Nuovo Observable Interno |
|---|---|
| `mergeAll` | Esegui concorrentemente |
| `concatAll` | Aggiungi alla coda (attendi completamento precedente) |
| `switchAll` | Annulla il vecchio e passa |
| `exhaustAll` | **Ignora (attendi completamento in esecuzione)** |

## ⚠️ Note Importanti

### Perdita di Eventi

`exhaustAll` **ignora completamente** gli eventi in esecuzione, quindi è inappropriato se vuoi elaborare tutti gli eventi.

```ts
// ❌ exhaustAll è inappropriato se vuoi registrare tutti i click
// ✅ Usa mergeAll o concatAll
```

### Feedback UI

È importante comunicare visivamente agli utenti che gli eventi vengono "ignorati".

```ts
// Disabilita pulsante
saveButton.disabled = true;

// Mostra messaggio toast
showToast('Elaborazione in corso. Attendere un momento.');
```

### Casi d'Uso Appropriati

#### `exhaustAll` è Ottimale per:
- Operazioni di login (prevenire invii duplicati)
- Operazioni di salvataggio (prevenire esecuzione duplicata)
- Animazioni (non avviare nuova animazione mentre è in esecuzione)

#### `exhaustAll` non è Appropriato per:
- Operazioni di ricerca (vuoi eseguire l'ultima ricerca → `switchAll`)
- Tutti gli eventi devono essere elaborati (→ `mergeAll` o `concatAll`)
