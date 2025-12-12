---
description: Casos de uso práticos para operadores de filtragem RxJS (debounceTime, throttleTime, distinctUntilChanged, filter, etc.) são explicados. Aprenda padrões práticos para extrair apenas os valores que você precisa de streams, como pesquisa em tempo real, rolagem infinita, controle de eventos de alta frequência, desduplicação, etc., com exemplos de código TypeScript. Você aprenderá técnicas de implementação úteis para tratamento de eventos de UI e otimização de desempenho.
---

# Casos de Uso Práticos

## Filtragem em Tempo Real de Entrada do Usuário para Pesquisa

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  debounceTime,
  distinctUntilChanged,
  filter,
} from 'rxjs';

// Construir UI
const searchInput = document.createElement('input');
searchInput.placeholder = 'Digite termo de pesquisa (3+ caracteres)';
document.body.appendChild(searchInput);

const resultsContainer = document.createElement('div');
document.body.appendChild(resultsContainer);

// Stream de eventos
fromEvent(searchInput, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((term) => term.length >= 3)
  )
  .subscribe((searchTerm) => {
    resultsContainer.innerHTML = `Iniciando pesquisa por "${searchTerm}"...`;
  });

```

- **Processa apenas entrada confirmada** em intervalos de 300ms.
- **Pesquisas são realizadas apenas quando 3 ou mais caracteres** são digitados.
- **Entradas consecutivas da mesma palavra** são ignoradas.


## Simulação de Rolagem Infinita

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  throttleTime,
  distinctUntilChanged,
  scan,
} from 'rxjs';

// Construir UI
const scrollArea = document.createElement('div');
scrollArea.style.height = '200px';
scrollArea.style.overflow = 'auto';
scrollArea.style.border = '1px solid #ccc';
document.body.appendChild(scrollArea);

const itemsList = document.createElement('div');
scrollArea.appendChild(itemsList);

// Adicionar dados iniciais
function addItems(page: number) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.textContent = `Item ${(page - 1) * 10 + i}`;
    itemsList.appendChild(item);
  }
}
addItems(1);

// Stream de evento de rolagem
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

- Quando a posição de rolagem atinge **80% ou mais**, os próximos itens são carregados.
- **Auto-carrega até 5 páginas**.
- **Eventos de rolagem** são controlados **a cada 200ms**.


## Resumo de Como Escolher Operadores de Filtragem

| O Que Você Quer Fazer | Operador | Descrição |
|:---|:---|:---|
| Passar apenas dados que correspondem à condição | `filter` | Filtragem mais básica |
| Obter apenas os primeiros itens | `take`, `first` | Limitar número de itens adquiridos |
| Esperar até entrada ser confirmada | `debounceTime` | Ideal para entrada de formulário |
| Processar apenas em intervalos fixos | `throttleTime` | Aplicar a scroll, resize, etc. |
| Ignorar valores consecutivos iguais | `distinctUntilChanged` | Prevenir reprocessamento desnecessário de dados idênticos |


## Resumo

- Operadores de filtragem são essenciais para controlar streams de dados.
- Eles não são apenas poderosos quando usados sozinhos, mas ainda mais quando **combinados**.
- Levam diretamente a **melhoria de eficiência e desempenho** em aplicações orientadas a eventos e desenvolvimento de UI.
