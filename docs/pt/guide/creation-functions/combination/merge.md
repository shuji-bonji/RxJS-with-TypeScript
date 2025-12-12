---
description: "A Creation Function merge inscreve em múltiplos Observables simultaneamente e mescla valores em tempo real: Essencial para integrar fluxos de eventos paralelos"
---

# merge - mesclar múltiplos fluxos simultaneamente

`merge` é uma Creation Function que inscreve em múltiplos Observables simultaneamente,
Creation Function que inscreve em múltiplos Observables simultaneamente e emite os valores conforme são emitidos de cada Observable.

## Sintaxe básica e uso

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Fluxo 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Fluxo 2: ${val}`),
  take(2)
);

merge(source1$, source2$).subscribe(console.log);
// Exemplo de saída:
// Fluxo 1: 0
// Fluxo 2: 0
// Fluxo 1: 1
// Fluxo 1: 2
// Fluxo 2: 1
```

- Inscreve em todos os Observables simultaneamente, e os valores fluirão **na ordem de emissão**.
- Não há garantia de ordem, e **depende** do momento de emissão de cada Observable.


[🌐 Documentação Oficial RxJS - `merge`](https://rxjs.dev/api/index/function/merge)

## Padrões típicos de utilização

- **Mesclar** múltiplos eventos assíncronos (por exemplo, entrada do usuário e notificações do backend)
- **Agregar múltiplos fluxos de dados em um único fluxo**.
- **Combinar fontes paralelas de informação, por exemplo, atualizações em tempo real e integração de polling**.

## Exemplos práticos de código (com UI)

Combina eventos de clique e timer em tempo real.

```ts
import { merge, fromEvent, timer } from 'rxjs';
import { map } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Exemplo prático de merge:</h3>';
document.body.appendChild(output);

// Criar elemento de botão
const button = document.createElement('button');
button.textContent = 'Clique para disparar evento';
document.body.appendChild(button);

// Fluxo de clique
const click$ = fromEvent(button, 'click').pipe(
  map(() => '✅ Clique do botão detectado')
);

// Fluxo de timer
const timer$ = timer(3000, 3000).pipe(
  map((val) => `⏰ Evento de timer (${val})`)
);

// merge e exibir
merge(click$, timer$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- **Clique em um botão e um evento é gerado** imediatamente,
- **Timer dispara um evento repetido** a cada 3 segundos.
- Você pode experimentar como dois tipos diferentes de Observables podem ser mesclados em **tempo real**.


## Operadores Relacionados

- **[mergeWith](/pt/guide/operators/combination/mergeWith)** - Versão Pipeable Operator (usado em pipeline)
- **[mergeMap](/pt/guide/operators/transformation/mergeMap)** - mapeia e concatena cada valor em paralelo
- **[concat](/pt/guide/creation-functions/combination/concat)** - Creation Function de concatenação sequencial
