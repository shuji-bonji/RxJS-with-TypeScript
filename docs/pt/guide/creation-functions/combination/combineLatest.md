---
description: "combineLatest combina os valores mais recentes de múltiplos Observables: Essencial para validação de formulários em tempo real, sincronização de estado e dados dependentes"
---

# combineLatest - combinar os valores mais recentes

`combineLatest` é uma Creation Function que **combina todos os valores mais recentes de múltiplos Observables**.
Sempre que um novo valor é emitido de qualquer um dos Observables de origem, o resultado de todos os valores mais recentes são combinados.

## Sintaxe básica e uso

```ts
import { combineLatest, of } from 'rxjs';

const obs1$ = of('A', 'B', 'C');
const obs2$ = of(1, 2, 3);

combineLatest([obs1$, obs2$]).subscribe(([val1, val2]) => {
  console.log(val1, val2);
});

// Saída:
// C 1
// C 2
// C 3
```

- Após cada Observable ter emitido **pelo menos um valor**, o valor combinado é emitido.
- Sempre que um novo valor chega para qualquer um dos dois, o par mais recente é emitido novamente.

[🌐 Documentação Oficial RxJS - `combineLatest`](https://rxjs.dev/api/index/function/combineLatest)


## Padrões típicos de utilização

- **Validação em tempo real de entradas de formulário** (por exemplo, monitoramento simultâneo de nome e endereço de e-mail)
- **Sincronização de estado de múltiplos fluxos** (por exemplo, integração de valores de sensores e status de dispositivo)
- **Busca de dados com dependências** (por exemplo, combinação de ID de usuário e ID de configuração)

## Exemplos práticos de código (com UI)

Sempre combina e exibe o status mais recente dos dois campos de entrada de um formulário.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Exemplo prático de combineLatest:</h3>';
document.body.appendChild(output);

// Criar campos de formulário
const nameInput = document.createElement('input');
nameInput.placeholder = 'Digite o nome';
document.body.appendChild(nameInput);

const emailInput = document.createElement('input');
emailInput.placeholder = 'Digite o email';
document.body.appendChild(emailInput);

// Observable de cada entrada
const name$ = fromEvent(nameInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

const email$ = fromEvent(emailInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

// Combinar os valores de entrada mais recentes
combineLatest([name$, email$]).subscribe(([name, email]) => {
  output.innerHTML = `
    <div><strong>Nome:</strong> ${name}</div>
    <div><strong>Email:</strong> ${email}</div>
  `;
});
```

- Quando você digita em qualquer campo, os **dois estados de entrada mais recentes** são imediatamente exibidos.
- O `startWith('')` é usado para obter o resultado combinado desde o início.


## Operadores Relacionados

- **[combineLatestWith](/pt/guide/operators/combination/combineLatestWith)** - Versão Pipeable Operator (usado em pipeline)
- **[withLatestFrom](/pt/guide/operators/combination/withLatestFrom)** - dispara apenas o fluxo principal
- **[zip](/pt/guide/creation-functions/combination/zip)** - emparelha valores correspondentes Creation Function
