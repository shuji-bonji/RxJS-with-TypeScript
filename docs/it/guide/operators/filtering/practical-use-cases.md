---
description: Vengono spiegati i casi d'uso pratici degli operatori di filtraggio RxJS (debounceTime, throttleTime, distinctUntilChanged, filter, ecc.). Impara pattern pratici per estrarre solo i valori necessari dagli stream, come ricerca real-time, scrolling infinito, controllo eventi ad alta frequenza, deduplicazione, ecc., con esempi di codice TypeScript. Imparerai tecniche di implementazione utili per la gestione eventi UI e l'ottimizzazione delle prestazioni.
---

# Casi d'Uso Pratici

## Filtraggio Ricerca Real-Time dell'Input Utente

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  debounceTime,
  distinctUntilChanged,
  filter,
} from 'rxjs';

// Costruisci UI
const searchInput = document.createElement('input');
searchInput.placeholder = 'Inserisci termine di ricerca (3+ caratteri)';
document.body.appendChild(searchInput);

const resultsContainer = document.createElement('div');
document.body.appendChild(resultsContainer);

// Stream di eventi
fromEvent(searchInput, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((term) => term.length >= 3)
  )
  .subscribe((searchTerm) => {
    resultsContainer.innerHTML = `Avvio ricerca per "${searchTerm}"...`;
  });

```

- **Elabora solo l'input confermato** a intervalli di 300ms.
- **Le ricerche vengono eseguite solo quando vengono inseriti 3 o più caratteri**.
- **Gli inserimenti consecutivi della stessa parola** vengono ignorati.


## Simulazione Scrolling Infinito

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  throttleTime,
  distinctUntilChanged,
  scan,
} from 'rxjs';

// Costruisci UI
const scrollArea = document.createElement('div');
scrollArea.style.height = '200px';
scrollArea.style.overflow = 'auto';
scrollArea.style.border = '1px solid #ccc';
document.body.appendChild(scrollArea);

const itemsList = document.createElement('div');
scrollArea.appendChild(itemsList);

// Aggiungi dati iniziali
function addItems(page: number) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.textContent = `Elemento ${(page - 1) * 10 + i}`;
    itemsList.appendChild(item);
  }
}
addItems(1);

// Stream eventi scroll
fromEvent(scrollArea, 'scroll')
  .pipe(
    throttleTime(200),
    map(() => ({
      scrollTop: scrollArea.scrollTop,
      scrollHeight: scrollArea.scrollHeight,
      clientHeight: scrollArea.clientHeight,
    })),
    map(
      ({ scrollTop, scrollHeight, clientHeight }) =>
        (scrollTop + clientHeight) / scrollHeight
    ),
    distinctUntilChanged(),
    filter((ratio) => ratio > 0.8),
    scan((page) => page + 1, 1),
    filter((page) => page <= 5)
  )
  .subscribe((page) => {
    addItems(page);
  });

```

- Quando la posizione di scroll raggiunge **l'80% o più**, vengono caricati gli elementi successivi.
- **Carica automaticamente fino a 5 pagine**.
- **Gli eventi scroll** vengono limitati **ogni 200ms**.


## Riepilogo di Come Scegliere gli Operatori di Filtraggio

| Cosa Vuoi Fare | Operatore | Descrizione |
|:---|:---|:---|
| Passare solo dati che soddisfano la condizione | `filter` | Filtraggio più basilare |
| Ottenere solo i primi elementi | `take`, `first` | Limita il numero di elementi acquisiti |
| Attendere fino a conferma input | `debounceTime` | Ideale per input form |
| Elaborare solo a intervalli fissi | `throttleTime` | Applicare a scroll, resize, ecc. |
| Ignorare valori consecutivi uguali | `distinctUntilChanged` | Prevenire rielaborazione inutile di dati identici |


## Riepilogo

- Gli operatori di filtraggio sono essenziali per controllare gli stream di dati.
- Non sono solo potenti quando usati singolarmente, ma ancora di più quando **combinati**.
- Portano direttamente a **maggiore efficienza e prestazioni** nelle applicazioni event-driven e nello sviluppo UI.
