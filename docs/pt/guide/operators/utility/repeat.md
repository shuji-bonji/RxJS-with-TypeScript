---
description: O operador repeat reexecuta todo o stream um número especificado de vezes após o Observable fonte completar com sucesso. Pode ser usado para polling periódico, animação repetitiva e outras situações que requerem controle diferente de retry.
---

# repeat - Repetir Stream

O operador `repeat` reexecuta todo o stream um número especificado de vezes após o Observable fonte ter **completado com sucesso**.
Isso é útil para processos de polling, animações repetidas e controles que são diferentes de novas tentativas.

## 🔰 Sintaxe Básica e Operação

O uso mais simples é configurar uma sequência de valores para repetir um determinado número de vezes.

```ts
import { of } from 'rxjs';
import { repeat } from 'rxjs';

of('A', 'B')
  .pipe(
    repeat(2) // Repetir sequência inteira 2 vezes (gerar 2 vezes no total)
  )
  .subscribe(console.log);
// Saída:
// A
// B
// A
// B
```

[🌐 Documentação Oficial do RxJS - repeat](https://rxjs.dev/api/index/function/repeat)

## 💡 Exemplo de Uso Típico

Por exemplo, é usado para processos de polling simples ou animações de exibição repetidas.

```ts
import { of } from 'rxjs';
import { tap, delay, repeat } from 'rxjs';

of('✅ Dados recuperados com sucesso')
  .pipe(
    tap(() => console.log('Solicitação iniciada')),
    delay(1000),
    repeat(3) // Repetir 3 vezes
  )
  .subscribe(console.log);
// Saída:
// Solicitação iniciada
// ✅ Dados recuperados com sucesso
// main.ts:6 Solicitação iniciada
// ✅ Dados recuperados com sucesso
// main.ts:6 Solicitação iniciada
// ✅ Dados recuperados com sucesso
```

Neste exemplo, "solicitação → recuperação de dados" é repetida três vezes a cada segundo.

## 🧪 Exemplo de Código Prático (com UI)

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs';

// Área de exibição de saída
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>Exemplo de repeat:</h3>';
document.body.appendChild(repeatOutput);

// Exibição de contagem de repetição
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `Contagem de repetição: ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// Área de saída de valores
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// Repetição de sequência
of('A', 'B', 'C')
  .pipe(
    tap(() => {
      repeatCount++;
      repeatCountDisplay.textContent = `Contagem de repetição: ${repeatCount}`;
    }),
    repeat(3)
  )
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Valor: ${val} (repetição ${repeatCount})`;
    valuesOutput.appendChild(valueItem);
  });

```

## ✅ Resumo

- `repeat` **reexecuta todo o Observable após conclusão bem-sucedida**
- Ao contrário de `retry`, **não reexecuta em caso de erro**
- Pode ser usado para animações repetitivas, como processos de polling e **placeholders piscantes**
